# Publishes RGB + Depth + CameraInfo from the Piper's wrist-mounted D435
# RealSense over ROS2, if the loaded Robot USD has one (see ROBOT_PRESETS /
# go2_with_mid360_and_piper.usd's Piper/camera_link). Mirrors mid360.py's
# shape (find_sensor / publish_to_ros2, called from go2_example.py) -- but
# unlike the Mid-360 (a real sensor prim, create_sensor() just picks its
# scan-pattern attrs), camera_link is *purely a mount point*: headless
# inspection confirmed no UsdGeom.Camera prim anywhere under the Piper
# mount, and none of realsense2_description's usual optical-frame child
# links (camera_color_optical_frame etc.) either. find_sensor() locates
# that mount link; publish_to_ros2() creates the actual Camera prim (once,
# reused across rebuilds) as its child and builds the OmniGraph.
#
# NOTE: the bundled Piper USD also has a *second*, similarly-named prim,
# .../Piper/link6/d435_camera_link -- a plain decorative Xform (no
# PhysicsRigidBodyAPI) a couple cm off from camera_link, seemingly a
# leftover/vestigial mesh-placement marker from the asset's import. It is
# NOT part of the PhysX articulation (piper.find_arm's articulation walk,
# and piper.publish_to_ros2's live TF publish, never touch it), unlike
# camera_link, which is a real rigid body wired into the articulation via
# a PhysicsFixedJoint (.../Piper/joints/camera_link_joint, link6->
# camera_link) -- and whose name matches what a real Piper+D435 URDF (and
# this project's MoveIt config) actually calls it. Use camera_link, not
# d435_camera_link.

import omni.graph.core as og
import omni.usd
from pxr import Gf, Usd, UsdGeom

from . import settings

GRAPH_PATH = "/World/Go2PiperRealsenseROS2"
CAMERA_PRIM_NAME = "D435Sensor"
MOUNT_LINK_NAME = "camera_link"
# camera_link's xform *parent* is the flat "Piper" mount Xform (all of
# link1..link8/camera_link are authored as its direct xform children, with
# the real kinematic parenting done by PhysicsJoint prims instead -- see
# module docstring), not its kinematic parent link6 -- so link6 has to be
# looked up as camera_link's *sibling* by name, not GetParent(). It's a
# name (not a guess) shared with the real Piper+D435 URDF/piper.srdf's tip
# link, deliberately used below as the TF parent instead of camera_link
# itself: see publish_to_ros2's docstring.
WRIST_LINK_NAME = "link6"

# Rotation from the Camera prim's local frame (USD/Hydra convention: looks
# down local -Z, +Y is image-up, +X is image-right) into d435_camera_link's
# local frame.  The D435 mesh frame has the correct forward direction but the
# image axes are rolled by 180 degrees, so roll the camera about its optical
# axis.  RGB and depth share this camera/render product and are corrected
# together.
_CAMERA_ORIENT = Gf.Quatd(0.5, 0.5, -0.5, -0.5)  # (w, i, j, k)

# RGB and Depth share this one render product/Camera prim (see
# publish_to_ros2's docstring), but Intel's own D435 product brief specs
# them with *different* FOVs: RGB is 69.4 x 42.5 x 77 (+/-3deg) while Depth
# (the stream that actually drives the point cloud we're checking against
# the Mid-360) is wider, 85.2 x 58 (+/-3deg) at 16:9. Since only one FOV is
# achievable here, this is tuned to Depth's 85.2deg horizontal (at USD's
# default-ish horizontal aperture) -- the RGB image consequently renders
# somewhat wider than a real D435 color frame would, the opposite tradeoff
# from what an earlier version of this file had (tuned to RGB, leaving the
# point cloud narrower -- i.e. less coverage -- than real Depth data).
_HORIZONTAL_APERTURE_MM = 20.955
_FOCAL_LENGTH_MM = 11.4
# The product brief's raw hardware limits (Minimum Depth Distance/Min-Z:
# 0.105m, Maximum Range: "10m+, varies depending on performance accuracy,
# scene and light conditions") are best-case numbers, not what the sensor
# usably returns -- Intel's own spec page lists a separate, more
# conservative "Operating Range (Min-Max): ~0.3m - 3m" field for that,
# which is what this is set to: Isaac's simulated depth has no IR/ambient-
# light falloff or stereo-matching noise, so clipping to the raw 10m
# hardware limit made the point cloud read as unrealistically far-reaching
# (e.g. cleanly picking up walls several meters out) compared to a real
# D435 in the same room.
_CLIPPING_RANGE = (0.3, 3.0)

# ROS2CameraHelper's "depth_pcl" unprojects pixels with no pose input at all
# (isaacsim.core.nodes.IsaacConvertDepthToPointCloud takes only focal
# length/aperture/width/height/the raw depth buffer) -- it always emits
# points in REP-103 *optical*-frame convention (+Z forward along the optical
# axis, +X right, +Y down), a fixed 180deg-about-local-X remap off of the
# Camera prim's own Hydra/USD axes (-Z forward, +Y up), independent of
# whatever orientation the prim itself carries (confirmed against
# isaacsim.sensors.camera.Camera's own R_U_TRANSFORM = diag(1,-1,-1), the
# same "USD camera -> ROS camera" convention fix). _CAMERA_ORIENT above is
# tuned only for the 2D RGB/Depth image layout -- which RViz's Image display
# never runs through TF -- so it does *not* itself encode this optical
# remap. Folded into the published d435_color_optical_frame TF (see
# publish_to_ros2) so the PointCloud2 actually lines up with the rest of the
# tree (e.g. the Mid-360's lidar points) instead of landing 180deg off
# around the camera's local X (forward/back and up/down both flipped).
_ROS_OPTICAL_FIX = Gf.Quatd(0.0, 1.0, 0.0, 0.0)  # 180deg about local X, (w, i, j, k)


def find_sensor(robot_prim_path: str, mount_name: str = "Piper"):
    """Looks for the D435 mount link (camera_link, a real PhysX rigid body
    -- see module docstring) under the loaded robot's Piper arm -- same
    sibling-of-base scoping piper.find_arm uses (see its docstring):
    camera_link lives under the Piper mount Xform, which is a *sibling* of
    robot_prim_path (Go2's `base`), not a descendant of it. Returns None if
    the loaded Robot USD has no Piper mounted, or the mounted Piper has no
    camera link (e.g. a non-camera Piper variant)."""
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        return None
    parent_prim = robot_prim.GetParent()
    if not parent_prim.IsValid():
        return None
    mount_prim = stage.GetPrimAtPath(parent_prim.GetPath().AppendChild(mount_name))
    if not mount_prim.IsValid():
        return None
    for prim in Usd.PrimRange(mount_prim):
        if prim.GetName() == MOUNT_LINK_NAME:
            return prim
    return None


def _relative_transform(prim, reference_prim, local_rotation_fix=None):
    """prim's translation/rotation relative to reference_prim, computed from
    the actual USD hierarchy -- mirrors mid360.py's _relative_transform
    (same reasoning: not a guessed offset). local_rotation_fix, if given, is
    applied to prim's *local* axes first (see _ROS_OPTICAL_FIX above) --
    i.e. the returned pose is reference_prim -> (prim's frame, relabeled by
    that rotation), not reference_prim -> prim's own raw frame."""
    prim_to_world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    if local_rotation_fix is not None:
        prim_to_world = Gf.Matrix4d(1.0).SetRotateOnly(local_rotation_fix) * prim_to_world
    reference_to_world = UsdGeom.Xformable(reference_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    prim_to_reference = prim_to_world * reference_to_world.GetInverse()
    translation = prim_to_reference.ExtractTranslation()
    quat = prim_to_reference.ExtractRotationQuat()
    imaginary = quat.GetImaginary()
    # ROS2PublishRawTransformTree expects (x, y, z, w), not Gf's (w, x, y, z).
    return translation, (imaginary[0], imaginary[1], imaginary[2], quat.GetReal())


def _find_or_create_camera(mount_link_prim):
    """Creates the Camera prim as a child of mount_link_prim on first call;
    reuses (and re-applies the fixed values to) it on later ones, so
    repeated re-Loads don't accumulate duplicate camera prims."""
    stage = mount_link_prim.GetStage()
    camera_path = mount_link_prim.GetPath().AppendChild(CAMERA_PRIM_NAME)
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera.ClearXformOpOrder()
    camera.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(_CAMERA_ORIENT)
    camera.GetHorizontalApertureAttr().Set(_HORIZONTAL_APERTURE_MM)
    camera.GetFocalLengthAttr().Set(_FOCAL_LENGTH_MM)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(*_CLIPPING_RANGE))
    return camera.GetPrim()


def publish_to_ros2(mount_link_prim) -> None:
    """Builds the OmniGraph publishing the D435's rgb/depth/camera_info/
    pointcloud over ROS2. mount_link_prim is find_sensor()'s return value
    (d435_camera_link). RGB, Depth and the point cloud all share one
    IsaacCreateRenderProduct (one simulated camera), so they're inherently
    pixel-aligned and one CameraInfo covers all of them. The point cloud is
    ROS2CameraHelper's "depth_pcl" type -- points reprojected from that same
    Depth stream and colorized from RGB, matching real realsense2_camera's
    depth/color/points topic (sensor_msgs/PointCloud2) when
    pointcloud.enable:=true.

    Also publishes a fixed link6->optical-frame TF, folding in camera_link's
    own (also fixed) offset from link6: this is deliberately parented at
    link6, not camera_link, even though camera_link is the actual mount --
    link6 is the name a *real* Piper URDF's robot_state_publisher also
    knows (piper.srdf's "arm" group tip_link), so this TF edge attaches
    correctly regardless of whether piper.py's own "piper_publish_arm_tf"
    Preference is publishing link6 (bare Isaac Sim) or a real
    robot_state_publisher is (piper_isaacsim_bringup's MoveIt stack) --
    camera_link itself is Isaac-only (see module docstring) and, worse,
    would conflict with a same-named PhysX link if it ever were published
    by both at once. Either way this is one fixed offset (camera_link
    doesn't move relative to link6 -- see camera_link_joint) so it only
    needs computing once, same as mid360.py's rigid chassis->lidar mount."""
    camera_prim = _find_or_create_camera(mount_link_prim)
    wrist_link_prim = mount_link_prim.GetParent().GetChild(WRIST_LINK_NAME)
    link_frame = wrist_link_prim.GetName()
    # frame_id's actual data (RGB/Depth images *and* the depth_pcl point
    # cloud) is in the optical-frame axes _ROS_OPTICAL_FIX defines, not the
    # Camera prim's own raw Hydra axes -- see that constant's comment.
    translation, orientation = _relative_transform(camera_prim, wrist_link_prim, _ROS_OPTICAL_FIX)

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)

    node_namespace = settings.get("ros2_namespace")
    domain_id = settings.get("ros2_domain_id")
    camera_prim_path = camera_prim.GetPath().pathString
    frame_id = settings.get("realsense_frame_id")
    width = int(settings.get("realsense_width"))
    height = int(settings.get("realsense_height"))

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishPointCloud", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishCameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("TFLinkToCamera", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
                ("CreateRenderProduct.inputs:cameraPrim", camera_prim_path),
                ("CreateRenderProduct.inputs:width", width),
                ("CreateRenderProduct.inputs:height", height),
                ("PublishRgb.inputs:type", "rgb"),
                ("PublishRgb.inputs:topicName", settings.get("realsense_rgb_topic")),
                ("PublishRgb.inputs:frameId", frame_id),
                ("PublishRgb.inputs:nodeNamespace", node_namespace),
                ("PublishDepth.inputs:type", "depth"),
                ("PublishDepth.inputs:topicName", settings.get("realsense_depth_topic")),
                ("PublishDepth.inputs:frameId", frame_id),
                ("PublishDepth.inputs:nodeNamespace", node_namespace),
                ("PublishPointCloud.inputs:type", "depth_pcl"),
                ("PublishPointCloud.inputs:topicName", settings.get("realsense_pointcloud_topic")),
                ("PublishPointCloud.inputs:frameId", frame_id),
                ("PublishPointCloud.inputs:nodeNamespace", node_namespace),
                ("PublishCameraInfo.inputs:topicName", settings.get("realsense_camera_info_topic")),
                ("PublishCameraInfo.inputs:frameId", frame_id),
                ("PublishCameraInfo.inputs:nodeNamespace", node_namespace),
                ("TFLinkToCamera.inputs:parentFrameId", link_frame),
                ("TFLinkToCamera.inputs:childFrameId", frame_id),
                ("TFLinkToCamera.inputs:translation", translation),
                ("TFLinkToCamera.inputs:rotation", orientation),
                # Not staticPublisher=True -- see mid360.py's identical note:
                # publishing every tick on the same "tf" topic as
                # piper.py's own live arm tree keeps this edge connected to
                # it in RViz.
                ("TFLinkToCamera.inputs:topicName", settings.get("ros2_tf_topic")),
                ("TFLinkToCamera.inputs:nodeNamespace", node_namespace),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishRgb.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishDepth.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishPointCloud.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishCameraInfo.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "PublishRgb.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "PublishDepth.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "PublishPointCloud.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "PublishCameraInfo.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishRgb.inputs:context"),
                ("Context.outputs:context", "PublishDepth.inputs:context"),
                ("Context.outputs:context", "PublishPointCloud.inputs:context"),
                ("Context.outputs:context", "PublishCameraInfo.inputs:context"),
                ("OnPlaybackTick.outputs:tick", "TFLinkToCamera.inputs:execIn"),
                ("Context.outputs:context", "TFLinkToCamera.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "TFLinkToCamera.inputs:timeStamp"),
            ],
        },
    )

    if domain_id:
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:domain_id").set(int(domain_id))
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:useDomainIDEnvVar").set(False)
