# Loads the "environment_usd_path" Preference (or the default ground plane)
# into a World -- shared by go2_example.py and piper_example.py so both
# examples pick up the same Environment USD / Preset the same way.

import carb

from . import settings


def load(world, log_prefix: str) -> None:
    environment_usd_path = settings.get("environment_usd_path")
    if environment_usd_path:
        from isaacsim.core.utils.stage import add_reference_to_stage

        # Preset paths (see settings.ENVIRONMENT_PRESETS) are Nucleus-relative
        # ("/Isaac/Environments/...") -- resolve against the assets root.
        # A manually typed/browsed path is already a full local/Nucleus path.
        if environment_usd_path.startswith("/Isaac/"):
            from isaacsim.storage.native import get_assets_root_path

            assets_root_path = get_assets_root_path()
            if assets_root_path is None:
                carb.log_error(
                    f"{log_prefix}: could not resolve Isaac Sim's assets root to load the "
                    f"environment preset '{environment_usd_path}'. Falling back to the default ground plane."
                )
                environment_usd_path = None
            else:
                environment_usd_path = assets_root_path + environment_usd_path

    if environment_usd_path:
        add_reference_to_stage(environment_usd_path, "/World/Environment")
        _add_default_lights_if_unlit(world.stage)
    else:
        load_ground_plane(world)


def load_ground_plane(world) -> None:
    """Just the default flat ground plane (+ lights), ignoring the
    "environment_usd_path" Preference."""
    world.scene.add_default_ground_plane(
        z_position=0,
        name="default_ground_plane",
        prim_path="/World/defaultGroundPlane",
        static_friction=0.2,
        dynamic_friction=0.2,
        restitution=0.01,
    )
    _add_fill_light(world.stage)


def _add_fill_light(stage) -> None:
    """The default ground plane's own light is a single overhead
    SphereLight with no ambient term, so every shadow under it renders pure
    black -- the arm's shadow on the floor, the underside of each lemon --
    in the viewport and in the D435's RGB alike. A dome light fills those
    in (compared headlessly against the D435's actual RGB output)."""
    from pxr import UsdLux

    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeFill")
    dome.CreateIntensityAttr(1000.0)


def _add_default_lights_if_unlit(stage) -> None:
    """The default ground plane (Isaac's default_environment asset) brings
    its own SphereLight, but the bundled Lemon Tree world (a bare
    GroundPlane + prop) and many custom environments carry no light at all,
    leaving the stage with none: the viewport's
    "Stage Lights" mode renders black, and more importantly so does every
    Camera render product (the D435's RGB stream -- depth is unaffected).
    The viewport's "Camera Light" mode only lights the *viewport*, not
    sensor cameras. Adds a dome (ambient fill) + distant (sun, for shading/
    shadows) light unless the loaded environment already brought its own
    (e.g. the Nucleus Warehouse/Simple Room presets)."""
    from pxr import Gf, UsdGeom, UsdLux

    for prim in stage.Traverse():
        if prim.HasAPI(UsdLux.LightAPI):
            return
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
    dome.CreateIntensityAttr(1000.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/DistantLight")
    sun.CreateIntensityAttr(2500.0)
    sun.CreateAngleAttr(1.0)
    # Tilted ~45deg off vertical so objects get shading/shadows, not a
    # flat top-down wash.
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(45.0, 0.0, 45.0))
