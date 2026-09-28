#!/usr/bin/env python3
# Builds data/Environments/LemonTree/lemon_tree_world.usd: a plain physics
# ground plane (isaacsim.core.api.objects.GroundPlane -- the same thing
# World.scene.add_default_ground_plane() creates, just usable without a
# World/BaseSample here) plus a lemon tree prop, referenced *directly* from
# an FBX file -- omni.kit.asset_converter registers a native "omniasset"
# SdfFileFormat plugin for .fbx (also .obj/.gltf/.glb/.md5/.lxo) once
# enabled, so AddReference() on an .fbx path composes it straight in, no
# explicit convert-to-USD step needed.
#
# Run with Isaac Sim's own Python (needs the isaacsim package + pxr):
#   ./python.sh tools/build_lemon_tree_world.py
# from the Isaac Sim install directory, or via the isaac_run/isaac_go2_run
# shell function documented in docs/README.md:
#   isaac_run tools/build_lemon_tree_world.py
#
# Source asset: a "Mini Lemon Tree" model, originally downloaded as both
# ~/Downloads/Mini+Lemon+Tree+obj.obj and ...+fbx.FBX, copied into this same
# output directory (data/Environments/LemonTree/mini_lemon_tree.fbx) so the
# built USD's reference is relative and travels with the repo -- referencing
# the Downloads copy directly would bake in an absolute path that breaks the
# moment that file moves or a clone runs this on a different machine.
#
# FBX over the plain OBJ specifically: the OBJ imports as one big untextured
# mesh with a single flat material and no per-part breakdown at all, but the
# FBX carries 6 *named* material GeomSubsets (leaf_Mat/branch_Mat/bark_Mat/
# grounde/pot/lemon_Mat) -- even though the actual texture files it
# references (pot.jpg, grounde.jpg, etc.) aren't included in either download
# (they're absolute Windows paths into the original author's own
# "D:\...\Mini Lemon Tree\Maps\" folder, confirmed missing here), those
# *named* subsets are still enough to hand-assign a plausible color per part
# below (real texture-based coloring is not recoverable without that Maps
# folder).
#
# If something goes wrong, set BUILD_LEMON_TREE_DEBUG_LOG=/some/path.log
# before running for a step-by-step trace written straight to a file --
# plain print() output from a script like this is not reliably captured
# when piping Isaac Sim's headless stdout (see the Mid-360 build script's
# own note on this).

import os

_DEBUG_LOG = os.environ.get("BUILD_LEMON_TREE_DEBUG_LOG")


def _debug(msg):
    if _DEBUG_LOG:
        with open(_DEBUG_LOG, "a") as f:
            f.write(str(msg) + "\n")


from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})
_debug("simulation app created")

import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdShade

from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.asset_converter")
simulation_app.update()

EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OUTPUT_DIR = os.path.join(EXT_ROOT, "data", "Environments", "LemonTree")
OUTPUT_PATH = os.path.join(OUTPUT_DIR, "lemon_tree_world.usd")
SOURCE_OBJ_PATH = os.environ.get("LEMON_TREE_SOURCE_OBJ", os.path.join(OUTPUT_DIR, "mini_lemon_tree.fbx"))
TREE_PATH = "/World/LemonTree"
TARGET_HEIGHT_M = 0.4  # a small tabletop/bench-scale "mini" tree
# This export comes in lying on its side -- 90deg about local X stands it
# upright. Flip the sign (-90.0) if it lands upside-down/backwards instead.
ROTATE_X_DEG = 90.0
# go2_example.py spawns Go2 at (0, 0, 0.42) -- offset well clear of the
# chassis+leg footprint so Load doesn't spawn the robot standing inside the
# pot. Still close enough for the Piper arm's own short reach once that's
# the actual test (see docs/README.md).
# Z=0.5: raises the tree's own base 0.5m off the ground plane (e.g. onto a
# stand/table height) so the lemons sit within the Piper arm's reach instead
# of near ground level -- _orient_scale_and_ground() otherwise always grounds
# the tree's lowest point at Z=0, so this is the only place to lift it.
POSITION_OFFSET = Gf.Vec3d(1.2, 0.0, 0.5)

# Hand-picked replacements for the FBX's own materials -- all imported as a
# flat, textureless mid-gray (diffuse_color_constant ~0.3-0.47, evidently
# the converter's fallback for an unresolvable texture reference, not a
# real authored color). Keyed by material prim name (case-sensitive, must
# match the FBX's own names exactly -- see module docstring).
MATERIAL_COLORS = {
    "leaf_Mat": Gf.Vec3f(0.16, 0.42, 0.14),
    "branch_Mat": Gf.Vec3f(0.32, 0.21, 0.12),
    "bark_Mat": Gf.Vec3f(0.24, 0.15, 0.09),
    "grounde": Gf.Vec3f(0.28, 0.19, 0.12),
    "pot": Gf.Vec3f(0.64, 0.34, 0.21),
    "lemon_Mat": Gf.Vec3f(0.85, 0.78, 0.13),
}


def _recolor_materials(stage, prim) -> None:
    """Overrides each of MATERIAL_COLORS' shaders' diffuse_color_constant
    input in place -- these are OmniPBR-style MDL shaders (see the "inputs"
    dumped by inspecting the imported FBX live), so the input is a plain
    Color3f UsdShade input, not a full material rebuild."""
    recolored = set()
    for shader_prim in Usd.PrimRange(prim):
        if not shader_prim.IsA(UsdShade.Shader):
            continue
        material_name = shader_prim.GetParent().GetName()
        color = MATERIAL_COLORS.get(material_name)
        if color is None:
            continue
        shader = UsdShade.Shader(shader_prim)
        color_input = shader.GetInput("diffuse_color_constant")
        if not color_input:
            continue
        color_input.Set(color)
        recolored.add(material_name)
    _debug(f"recolored materials: {sorted(recolored)}")
    missing = set(MATERIAL_COLORS) - recolored
    if missing:
        print(f"WARNING: expected materials not found/recolored: {sorted(missing)}")
        _debug(f"WARNING missing materials: {sorted(missing)}")


def _orient_scale_and_ground(prim) -> None:
    """Rotates prim upright (ROTATE_X_DEG), then scales+positions it against
    the *rotated* bounding box -- recomputed live via BBoxCache rather than
    hand-swapping Y/Z math for the rotation, so this stays correct even if
    ROTATE_X_DEG changes. Final placement: TARGET_HEIGHT_M tall, its lowest
    point POSITION_OFFSET.z above the ground plane, centered at
    POSITION_OFFSET in X/Y.

    xformOpOrder composition note (confirmed empirically -- see the repo's
    session notes): USD applies the *last*-listed op to the local point
    *first*, then works outward to the *first*-listed op last. So appending
    Scale then Translate (as an earlier version of this function did) puts
    Translate innermost, meaning it gets multiplied by the *subsequent*
    Scale op too -- a `scale` this small (~0.0008, see below) silently
    shrinks any translate distance down to a fraction of a millimeter.
    The op order is rebuilt from scratch below as
    [Translate, Scale, RotateX] (first-to-last / outermost-to-innermost) so
    Translate is applied last, in already-scaled, real-world meters.
    """
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddRotateXOp(UsdGeom.XformOp.PrecisionDouble).Set(ROTATE_X_DEG)

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=False)
    bbox = bbox_cache.ComputeWorldBound(prim)
    aligned_range = bbox.ComputeAlignedRange()
    bbox_min, bbox_max = aligned_range.GetMin(), aligned_range.GetMax()
    _debug(f"post-rotation bbox: min={bbox_min} max={bbox_max}")
    raw_height = bbox_max[2] - bbox_min[2]
    assert raw_height > 0, f"degenerate bounding box (raw_height={raw_height}) -- did the reference resolve?"
    scale = TARGET_HEIGHT_M / raw_height
    print(f"post-rotation height={raw_height:.4f} (unknown units) -> scale={scale:.6f} for target height {TARGET_HEIGHT_M}m")
    _debug(f"raw_height={raw_height} scale={scale}")

    # bbox_min/max were measured post-rotation but pre-scale, so they're
    # already in this prim's (rotated) local-ish units -- scale them down to
    # stage units here rather than re-measuring a 3rd time. Center X/Y on
    # POSITION_OFFSET, drop Z so the lowest point lands at POSITION_OFFSET.z.
    center_x = (bbox_min[0] + bbox_max[0]) / 2 * scale
    center_y = (bbox_min[1] + bbox_max[1]) / 2 * scale
    ground_z = bbox_min[2] * scale
    translate_value = POSITION_OFFSET + Gf.Vec3d(-center_x, -center_y, -ground_z)

    # Rebuild in the corrected order -- see the docstring above. The RotateX
    # value is the same ROTATE_X_DEG used for the measurement pass; only the
    # *order* relative to Scale/Translate changes here.
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(translate_value)
    xformable.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(scale, scale, scale))
    xformable.AddRotateXOp(UsdGeom.XformOp.PrecisionDouble).Set(ROTATE_X_DEG)


def main():
    assert os.path.isfile(SOURCE_OBJ_PATH), f"source asset not found: {SOURCE_OBJ_PATH}"
    print(f"source asset: {SOURCE_OBJ_PATH}")
    _debug(f"source asset: {SOURCE_OBJ_PATH}")

    # Same temp-file-then-replace pattern as the Mid-360/Piper build
    # scripts -- see build_go2_with_mid360.py's comment on why.
    temp_path = OUTPUT_PATH + ".building.usd"
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    usd_context = omni.usd.get_context()
    usd_context.new_stage()
    simulation_app.update()
    # Anchor the stage at (a temp file next to) the real output path
    # *before* adding the asset reference, so a relative "./mini_lemon_tree.fbx"
    # resolves (and serializes) relative to this file's own directory --
    # portable across clones of this repo at any path, same reasoning as
    # build_go2_with_mid360.py's own identical comment.
    assert usd_context.save_as_stage(temp_path)
    simulation_app.update()
    stage = usd_context.get_stage()

    world_prim = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(world_prim)

    GroundPlane(prim_path="/World/GroundPlane", z_position=0)
    simulation_app.update()

    asset_ref = "./" + os.path.relpath(SOURCE_OBJ_PATH, OUTPUT_DIR)
    _debug(f"asset_ref: {asset_ref}")
    tree_prim = stage.DefinePrim(TREE_PATH, "Xform")
    tree_prim.GetReferences().AddReference(asset_ref)
    simulation_app.update()
    simulation_app.update()
    _debug(f"tree_prim children: {[c.GetPath().pathString for c in tree_prim.GetChildren()]}")
    assert list(tree_prim.GetChildren()), f"asset reference did not compose anything under {TREE_PATH}"

    _recolor_materials(stage, tree_prim)
    _orient_scale_and_ground(tree_prim)
    simulation_app.update()

    save_ok = usd_context.save_stage()
    _debug(f"save_stage() returned: {save_ok}")
    assert save_ok
    usd_context.close_stage()
    simulation_app.update()
    os.replace(temp_path, OUTPUT_PATH)
    print(f"Saved {OUTPUT_PATH}")


try:
    main()
    _debug("main() completed without exception")
except Exception as e:
    _debug(f"main() raised: {type(e).__name__}: {e}")
    raise
finally:
    temp_path = OUTPUT_PATH + ".building.usd"
    if os.path.isfile(temp_path):
        _debug(f"finally: removing leftover temp file {temp_path}")
        os.remove(temp_path)
    simulation_app.close()
    _debug("simulation_app closed")
