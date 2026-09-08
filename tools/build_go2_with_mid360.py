#!/usr/bin/env python3
# Rebuilds data/Robots/Go2/usd/go2_with_mid360.usd from go2.usd +
# data/Sensors/Mid360/Mid360.usd (the Mid-360 CAD model) + a native Mid-360
# lidar sensor (see go2_in_isaacsim/mid360.py -- omni:sensor:Core:* attrs,
# NOT the older Camera+sensorModelConfig JSON-lookup approach, which does
# not actually produce any points in this Isaac Sim version).
#
# Run with Isaac Sim's own Python (needs the isaacsim package + pxr):
#   ./python.sh tools/build_go2_with_mid360.py
# from the Isaac Sim install directory, or via the isaac_run/isaac_go2_run
# shell function documented in docs/README.md:
#   isaac_run tools/build_go2_with_mid360.py
#
# Re-run this any time data/Sensors/Mid360/Mid360.usd's own coordinate-fix
# changes, or to move the mount -- it preserves the existing file's Mid360
# mount Xform's transform (translate/rotate -- the real-world mount
# offset/tilt, measured against the physical robot) if the file already
# exists, so you don't need to remember or hardcode those numbers here.
#
# If something goes wrong, set BUILD_MID360_DEBUG_LOG=/some/path.log before
# running for a step-by-step trace written straight to a file -- plain
# print() output from a script like this is not reliably captured when
# piping Isaac Sim's headless stdout (Kit's own logging pipeline can drop
# unflushed buffers on shutdown, which cost real debugging time here once).

import os
import sys

_DEBUG_LOG = os.environ.get("BUILD_MID360_DEBUG_LOG")


def _debug(msg):
    if _DEBUG_LOG:
        with open(_DEBUG_LOG, "a") as f:
            f.write(str(msg) + "\n")


from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})
_debug("simulation app created")

import omni.usd
from pxr import Gf, Usd, UsdGeom

EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, EXT_ROOT)

from isaacsim.core.utils.extensions import enable_extension

enable_extension("go2_in_isaacsim")
enable_extension("isaacsim.sensors.rtx")
simulation_app.update()

from go2_in_isaacsim import mid360

OUTPUT_PATH = os.path.join(EXT_ROOT, "data", "Robots", "Go2", "usd", "go2_with_mid360.usd")
GO2_REF = "./go2.usd"
MID360_REF = "../../../Sensors/Mid360/Mid360.usd"
MOUNT_PATH = "/go2_description/base/Mid360"
DEFAULT_MOUNT_TRANSLATE = Gf.Vec3d(0.28, 0.0, 0.10)
DEFAULT_MOUNT_ROTATE_ZYX = Gf.Vec3d(0.0, 0.0, 0.0)


def _read_existing_mount_transform():
    """If go2_with_mid360.usd already exists, read the Mid360 mount Xform's
    current translate/rotateZYX ops (the real-world offset/tilt someone
    measured against the physical robot and set via the Isaac Sim UI) so
    rebuilding doesn't silently discard that adjustment."""
    if not os.path.isfile(OUTPUT_PATH):
        return DEFAULT_MOUNT_TRANSLATE, DEFAULT_MOUNT_ROTATE_ZYX
    stage = Usd.Stage.Open(OUTPUT_PATH)
    prim = stage.GetPrimAtPath(MOUNT_PATH)
    if not prim.IsValid():
        return DEFAULT_MOUNT_TRANSLATE, DEFAULT_MOUNT_ROTATE_ZYX
    translate_attr = prim.GetAttribute("xformOp:translate")
    rotate_attr = prim.GetAttribute("xformOp:rotateZYX")
    translate = translate_attr.Get() if translate_attr.IsValid() else DEFAULT_MOUNT_TRANSLATE
    rotate = rotate_attr.Get() if rotate_attr.IsValid() else DEFAULT_MOUNT_ROTATE_ZYX
    return translate, rotate


def main():
    _debug(f"mid360 module file: {mid360.__file__}")
    _debug(f"create_sensor has orient fix: {'_SENSOR_ORIENT_FIX' in dir(mid360)}")
    mount_translate, mount_rotate_zyx = _read_existing_mount_transform()
    print(f"Mid360 mount transform: translate={mount_translate}  rotateZYX={mount_rotate_zyx}")
    _debug(f"Mid360 mount transform: translate={mount_translate}  rotateZYX={mount_rotate_zyx}")

    # Build into a temp file in the same directory (so relative references
    # still resolve/serialize the same as the real output path would), and
    # only replace the real file with it at the very end, on success. Saving
    # straight to OUTPUT_PATH is not safe: save_as_stage() below writes
    # immediately (an empty anchor stage at that point), so if anything
    # after it raises, OUTPUT_PATH is left holding that empty stage instead
    # of its previous (possibly hand-adjusted, uncommitted) content -- this
    # happened once already, losing a real edit that thankfully still lived
    # in git history from an earlier commit.
    temp_path = OUTPUT_PATH + ".building.usd"

    usd_context = omni.usd.get_context()
    usd_context.new_stage()
    simulation_app.update()

    # Anchor the stage at (a temp file next to) the real output path
    # *before* adding relative references, so "./go2.usd" and
    # "../../../Sensors/..." resolve (and serialize) relative to this file's
    # own directory -- portable across clones of this repo at any path.
    assert usd_context.save_as_stage(temp_path)
    simulation_app.update()
    stage = usd_context.get_stage()

    robot_prim = stage.DefinePrim("/go2_description", "Xform")
    robot_prim.GetReferences().AddReference(GO2_REF)
    stage.SetDefaultPrim(robot_prim)
    simulation_app.update()
    simulation_app.update()
    assert stage.GetPrimAtPath("/go2_description/base").IsValid(), "go2.usd reference did not compose 'base'"

    # Reference Mid360.usd *directly* onto the mount prim (not a child of
    # it): Mid360.usd's own defaultPrim ("/World") composes its "mid_360_asm"
    # child onto whatever prim references it, so the mount ends up with
    # "mid_360_asm" as its child automatically -- wrapping it in another
    # same-named prim here would double-nest it.
    mount_prim = stage.DefinePrim(MOUNT_PATH, "Xform")
    mount_prim.GetReferences().AddReference(MID360_REF)
    mount_xformable = UsdGeom.Xformable(mount_prim)
    mount_xformable.ClearXformOpOrder()
    mount_xformable.AddTranslateOp().Set(mount_translate)
    mount_xformable.AddRotateZYXOp().Set(mount_rotate_zyx)
    simulation_app.update()

    sensor_prim = mid360.create_sensor(name="Mid360Sensor", parent=MOUNT_PATH)
    _debug(f"sensor_prim: {sensor_prim}")
    assert sensor_prim is not None and sensor_prim.IsValid()
    expected_sensor_path = f"{MOUNT_PATH}/Mid360Sensor"
    assert sensor_prim.GetPath().pathString == expected_sensor_path, (
        f"sensor landed at {sensor_prim.GetPath()}, expected {expected_sensor_path} "
        "(the replicator-API path/parent handling flattened it again -- see the comment above)"
    )
    orient_attr = sensor_prim.GetAttribute("xformOp:orient")
    _debug(f"sensor orient after create_sensor(): valid={orient_attr.IsValid()} value={orient_attr.Get() if orient_attr.IsValid() else None}")
    print(f"sensor created at {sensor_prim.GetPath()}, type={sensor_prim.GetTypeName()}")

    save_ok = usd_context.save_stage()
    _debug(f"save_stage() returned: {save_ok}")
    assert save_ok
    usd_context.close_stage()
    simulation_app.update()
    _debug(f"about to replace {temp_path} -> {OUTPUT_PATH}")
    os.replace(temp_path, OUTPUT_PATH)
    _debug("replace done")
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
    else:
        _debug("finally: no leftover temp file")
    simulation_app.close()
    _debug("simulation_app closed")
