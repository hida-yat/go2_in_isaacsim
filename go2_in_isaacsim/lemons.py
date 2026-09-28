# Graspable lemons for piper_example.py's grasp-practice area: N physics
# rigid bodies scattered at random in front of the (fixed-base) Piper arm,
# re-scattered on every Reset or from the example panel's "Randomize
# Lemons" button.
#
# Not taken from data/Environments/LemonTree's FBX: that model's lemons are
# fused into the tree's own mesh (one GeomSubset, no separable prims), so
# they can't be individual rigid bodies. Each lemon here is instead a
# procedural ellipsoid mesh, sized to a real lemon and to fit the Piper
# gripper (2 x joint7/joint8's 0.035m stroke = ~70mm max opening).
#
# Collision is a convex hull of that mesh (not a UsdGeom.Sphere with
# non-uniform scale, which PhysX can't represent as a true sphere), and
# angular damping is raised so a lemon settles on its side instead of
# rolling away across the ground plane forever.

import math

import numpy as np
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade, Vt

LEMONS_ROOT = "/World/Lemons"

# Semi-axes (m): ~75mm long, ~55mm across -- a typical supermarket lemon.
_SEMI_AXES = (0.0375, 0.0275, 0.0275)
_MASS_KG = 0.1
_ANGULAR_DAMPING = 2.0
_FRICTION = 1.0
_COLOR = Gf.Vec3f(0.95, 0.82, 0.1)

# Spawn region, in the arm's own base frame (world, since the arm-only
# mode puts arm_base at the origin): an annular sector in front of the
# arm (+X), inside Piper's ~0.6m reach but clear of its own base/link1.
_SPAWN_RADIUS = (0.25, 0.45)
_SPAWN_YAW = (-math.radians(50.0), math.radians(50.0))
# Minimum center-to-center spacing, so lemons never spawn overlapping (and
# explode apart on the first physics step).
_MIN_SEPARATION = 0.1
_DROP_HEIGHT = 0.005


def _ellipsoid_mesh(stage, path: str, segments: int = 24, rings: int = 16):
    mesh = UsdGeom.Mesh.Define(stage, path)
    a, b, c = _SEMI_AXES
    points = [Gf.Vec3f(0.0, 0.0, c)]
    for i in range(1, rings):
        theta = math.pi * i / rings
        for j in range(segments):
            phi = 2.0 * math.pi * j / segments
            points.append(
                Gf.Vec3f(a * math.sin(theta) * math.cos(phi), b * math.sin(theta) * math.sin(phi), c * math.cos(theta))
            )
    points.append(Gf.Vec3f(0.0, 0.0, -c))
    bottom = len(points) - 1

    counts, indices = [], []

    def ring_vertex(ring, j):
        return 1 + (ring - 1) * segments + (j % segments)

    for j in range(segments):
        counts.append(3)
        indices += [0, ring_vertex(1, j), ring_vertex(1, j + 1)]
    for ring in range(1, rings - 1):
        for j in range(segments):
            counts.append(4)
            indices += [
                ring_vertex(ring, j),
                ring_vertex(ring + 1, j),
                ring_vertex(ring + 1, j + 1),
                ring_vertex(ring, j + 1),
            ]
    for j in range(segments):
        counts.append(3)
        indices += [bottom, ring_vertex(rings - 1, j + 1), ring_vertex(rings - 1, j)]

    mesh.CreatePointsAttr(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    # Analytic per-vertex ellipsoid normals (gradient of x²/a² + y²/b² +
    # z²/c²), so the lemon shades smoothly instead of showing its facets.
    normals = [Gf.Vec3f(p[0] / a**2, p[1] / b**2, p[2] / c**2).GetNormalized() for p in points]
    mesh.CreateNormalsAttr(Vt.Vec3fArray(normals))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(-a, -b, -c), Gf.Vec3f(a, b, c)]))
    return mesh


def _shared_materials(stage):
    """One visual + one physics material, shared by every lemon."""
    visual_path = f"{LEMONS_ROOT}/Looks/LemonVisual"
    visual = UsdShade.Material.Define(stage, visual_path)
    shader = UsdShade.Shader.Define(stage, f"{visual_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(_COLOR)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    visual.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    physics = UsdShade.Material.Define(stage, f"{LEMONS_ROOT}/Looks/LemonPhysics")
    physics_api = UsdPhysics.MaterialAPI.Apply(physics.GetPrim())
    physics_api.CreateStaticFrictionAttr(_FRICTION)
    physics_api.CreateDynamicFrictionAttr(_FRICTION)
    physics_api.CreateRestitutionAttr(0.0)
    return visual, physics


def spawn(stage, count: int) -> list:
    """(Re)creates count lemons under LEMONS_ROOT, all at a placeholder pose
    (call randomize() to scatter them). Returns their prim paths."""
    if stage.GetPrimAtPath(LEMONS_ROOT).IsValid():
        stage.RemovePrim(LEMONS_ROOT)
    UsdGeom.Scope.Define(stage, LEMONS_ROOT)
    visual, physics = _shared_materials(stage)

    paths = []
    for i in range(count):
        path = f"{LEMONS_ROOT}/Lemon_{i:02d}"
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp().Set(Gf.Vec3d(0.35, 0.0, _SEMI_AXES[2] + _DROP_HEIGHT + 0.1 * i))
        xform.AddOrientOp().Set(Gf.Quatf(1.0))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(_MASS_KG)
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateAngularDampingAttr(_ANGULAR_DAMPING)

        mesh = _ellipsoid_mesh(stage, f"{path}/mesh")
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(visual)
        UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(
            physics, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
        paths.append(path)
    return paths


def sample_poses(count: int, rng: np.random.Generator) -> list:
    """count non-overlapping (position, quaternion wxyz) poses in the spawn
    region, each lying on its side with a random heading."""
    positions = []
    attempts = 0
    while len(positions) < count:
        attempts += 1
        if attempts > 1000:
            raise RuntimeError(
                f"could not place {count} lemons {_MIN_SEPARATION}m apart in the spawn region -- lower the count"
            )
        radius = rng.uniform(*_SPAWN_RADIUS)
        yaw = rng.uniform(*_SPAWN_YAW)
        candidate = np.array([radius * math.cos(yaw), radius * math.sin(yaw), _SEMI_AXES[2] + _DROP_HEIGHT])
        if all(np.linalg.norm(candidate[:2] - p[:2]) >= _MIN_SEPARATION for p in positions):
            positions.append(candidate)

    poses = []
    for position in positions:
        heading = rng.uniform(-math.pi, math.pi)
        poses.append((position, np.array([math.cos(heading / 2), 0.0, 0.0, math.sin(heading / 2)])))
    return poses


def randomize(rigid_prims: list, rng: np.random.Generator) -> None:
    """Scatters rigid_prims (isaacsim.core.prims.SingleRigidPrim, one per
    spawn() path) to fresh random poses, at rest. Also makes those poses
    their default state, so a physics Stop/Play keeps this layout rather
    than snapping back to spawn()'s placeholder stack."""
    for prim, (position, orientation) in zip(rigid_prims, sample_poses(len(rigid_prims), rng)):
        prim.set_world_pose(position=position, orientation=orientation)
        prim.set_default_state(position=position, orientation=orientation)
        prim.set_linear_velocity(np.zeros(3))
        prim.set_angular_velocity(np.zeros(3))
