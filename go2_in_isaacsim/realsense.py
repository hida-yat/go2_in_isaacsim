# Publishes RGB + Depth + CameraInfo from the Piper's wrist-mounted D435
# RealSense over ROS2, if the loaded Robot USD has one (see ROBOT_PRESETS /
# go2_with_mid360_and_piper.usd's Piper/link6/d435_camera_link). Mirrors
# mid360.py's shape (find_sensor / publish_to_ros2, called from
# go2_example.py) -- but unlike the Mid-360 (a real sensor prim,
# create_sensor() just picks its scan-pattern attrs), the D435 mount in the
# bundled Piper USD is *purely decorative*: headless inspection confirmed no
# UsdGeom.Camera prim anywhere under the Piper mount, and none of
# realsense2_description's usual optical-frame child links
# (camera_color_optical_frame etc.) either -- just the mesh/joint mount,
# named d435_camera_link (a child of link6) with camera_bottom_screw_frame
# under it. find_sensor() locates that mount link; publish_to_ros2() creates
# the actual Camera prim (once, reused across rebuilds) as its child and
# builds the OmniGraph.

import omni.graph.core as og
import omni.usd
from pxr import Gf, Usd, UsdGeom

from . import settings

GRAPH_PATH = "/World/Go2PiperRealsenseROS2"
CAMERA_PRIM_NAME = "D435Sensor"
MOUNT_LINK_NAME = "d435_camera_link"

# Rotation from the Camera prim's local frame (USD/Hydra convention: looks
# down local -Z, +Y is image-up, +X is image-right) into d435_camera_link's
# local frame.  The D435 mesh frame has the correct forward direction but the
# image axes are rolled by 180 degrees, so roll the camera about its optical
# axis.  RGB and depth share this camera/render product and are corrected
# together.
_CAMERA_ORIENT = Gf.Quatd(0.5, 0.5, -0.5, -0.5)  # (w, i, j, k)

# D435 RGB stream spec is ~69deg horizontal FOV; focal length tuned for that
# at USD's default horizontal aperture. Depth spec is ~0.105m-10m; near clip
# loosened slightly to avoid clipping into the sim mesh itself.
_HORIZONTAL_APERTURE_MM = 20.955
_FOCAL_LENGTH_MM = 15.0
_CLIPPING_RANGE = (0.05, 10.0)


def find_sensor(robot_prim_path: str, mount_name: str = "Piper"):
    """Looks for the D435 mount link under the loaded robot's Piper arm --
    same sibling-of-base scoping piper.find_arm uses (see its docstring):
    d435_camera_link lives under the Piper mount Xform, which is a *sibling*
    of robot_prim_path (Go2's `base`), not a descendant of it. Returns None
    if the loaded Robot USD has no Piper mounted, or the mounted Piper has no
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
    """Builds the OmniGraph publishing the D435's rgb/depth/camera_info over
    ROS2. mount_link_prim is find_sensor()'s return value (d435_camera_link).
    RGB and Depth share one IsaacCreateRenderProduct (one simulated camera),
    so they're inherently pixel-aligned and one CameraInfo covers both."""
    camera_prim = _find_or_create_camera(mount_link_prim)

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
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishCameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
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
                ("PublishCameraInfo.inputs:topicName", settings.get("realsense_camera_info_topic")),
                ("PublishCameraInfo.inputs:frameId", frame_id),
                ("PublishCameraInfo.inputs:nodeNamespace", node_namespace),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishRgb.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishDepth.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishCameraInfo.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "PublishRgb.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "PublishDepth.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "PublishCameraInfo.inputs:renderProductPath"),
                ("Context.outputs:context", "PublishRgb.inputs:context"),
                ("Context.outputs:context", "PublishDepth.inputs:context"),
                ("Context.outputs:context", "PublishCameraInfo.inputs:context"),
            ],
        },
    )

    if domain_id:
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:domain_id").set(int(domain_id))
        og.Controller.attribute(f"{GRAPH_PATH}/Context.inputs:useDomainIDEnvVar").set(False)
