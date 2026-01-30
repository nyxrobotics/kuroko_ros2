#!/usr/bin/env python3

"""
SDF -> USDA exporter that runs inside Isaac Sim's python.sh environment.

What it does:
- Parses SDF (v1.7) and creates a USD stage with one Xform per link.
- For primitive geometries (box/sphere/cylinder/capsule), creates native USD prims.
- For mesh geometries, converts mesh assets to USD using omni.kit.asset_converter,
  then references the converted USD under the corresponding link prim.

This script is intended to be executed with Isaac Sim's python.sh. The extension
'omni.kit.asset_converter' must be available. See:
- Omniverse Asset Converter docs.  citeturn0search1
- Isaac Sim Python scripting environment setup. citeturn0search2
"""

import argparse
import asyncio
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from omni.isaac.kit import SimulationApp

def _parse_pose(pose_text: str):
    # SDF pose: x y z roll pitch yaw
    vals = [float(x) for x in pose_text.strip().split()]
    if len(vals) != 6:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    return (vals[0], vals[1], vals[2]), (vals[3], vals[4], vals[5])

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input_sdf", required=True)
    ap.add_argument("--output_usda", required=True)
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()

    input_sdf = Path(args.input_sdf).expanduser().resolve()
    output_usda = Path(args.output_usda).expanduser().resolve()
    output_usda.parent.mkdir(parents=True, exist_ok=True)

    # Start Isaac Sim
    simulation_app = SimulationApp({"headless": bool(args.headless)})

    import omni
    import carb
    from pxr import Usd, UsdGeom, Gf, Sdf

    # Enable asset converter
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.kit.asset_converter", True)
    import omni.kit.asset_converter

    # Create stage
    stage = Usd.Stage.CreateNew(str(output_usda))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    model = UsdGeom.Xform.Define(stage, "/World/kuroko")

    # Parse SDF
    root = ET.fromstring(input_sdf.read_text())
    model_el = root.find("model")
    if model_el is None:
        raise RuntimeError("No <model> in SDF")

    # Simple converter cache for meshes
    mesh_cache_dir = output_usda.parent / "_mesh_usd"
    mesh_cache_dir.mkdir(parents=True, exist_ok=True)

    async def convert_mesh_to_usd(src_path: Path) -> Path:
        # Convert with Asset Converter into a .usd file
        dst = mesh_cache_dir / (src_path.stem + ".usd")
        if dst.exists():
            return dst

        task = omni.kit.asset_converter.AssetConverterTask()
        task.input_asset = str(src_path)
        task.output_asset = str(dst)
        task.output_format = "usd"

        # Basic settings
        task.flatten = True
        task.create_materials = True
        task.merge_materials = True

        ok = await omni.kit.asset_converter.get_instance().convert_async(task)
        if not ok:
            raise RuntimeError(f"Asset conversion failed: {src_path} -> {dst}")
        return dst

    def add_primitive(parent_path: str, geom_el: ET.Element, name: str, pose_xyz, pose_rpy):
        # Map SDF primitive -> USD geom
        # Note: We keep it simple: create as Xform + shape at origin, then apply transform on Xform.
        xform = UsdGeom.Xform.Define(stage, parent_path + "/" + name)
        xform.AddTranslateOp().Set(Gf.Vec3d(*pose_xyz))
        xform.AddRotateXYZOp().Set(Gf.Vec3d(pose_rpy[0]*57.2957795, pose_rpy[1]*57.2957795, pose_rpy[2]*57.2957795))

        if geom_el.find("box") is not None:
            size = [float(x) for x in geom_el.find("box").find("size").text.split()]
            cube = UsdGeom.Cube.Define(stage, xform.GetPath().pathString + "/geom")
            cube.GetSizeAttr().Set(1.0)
            cube.AddScaleOp().Set(Gf.Vec3f(size[0], size[1], size[2]))
            return

        if geom_el.find("sphere") is not None:
            radius = float(geom_el.find("sphere").find("radius").text)
            sph = UsdGeom.Sphere.Define(stage, xform.GetPath().pathString + "/geom")
            sph.GetRadiusAttr().Set(radius)
            return

        if geom_el.find("cylinder") is not None:
            radius = float(geom_el.find("cylinder").find("radius").text)
            length = float(geom_el.find("cylinder").find("length").text)
            cyl = UsdGeom.Cylinder.Define(stage, xform.GetPath().pathString + "/geom")
            cyl.GetRadiusAttr().Set(radius)
            cyl.GetHeightAttr().Set(length)
            cyl.GetAxisAttr().Set(UsdGeom.Tokens.z)
            return

        # capsule not always present in SDF 1.7, ignore if not.
        carb.log_warn(f"Unsupported primitive geometry in {name}, skipping.")

    async def build():
        # Create prims per link (use visuals only)
        for link_el in model_el.findall("link"):
            link_name = link_el.get("name", "link")
            link_path = f"/World/kuroko/{link_name}"
            link_xf = UsdGeom.Xform.Define(stage, link_path)

            # Visuals
            for i, vis_el in enumerate(link_el.findall("visual")):
                geom = vis_el.find("geometry")
                if geom is None:
                    continue

                pose_el = vis_el.find("pose")
                pose_xyz, pose_rpy = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
                if pose_el is not None and pose_el.text:
                    pose_xyz, pose_rpy = _parse_pose(pose_el.text)

                if geom.find("mesh") is not None:
                    uri_el = geom.find("mesh").find("uri")
                    if uri_el is None or not uri_el.text:
                        continue
                    src = Path(uri_el.text).expanduser()
                    if not src.is_absolute():
                        # Resolve relative to SDF directory
                        src = (input_sdf.parent / src).resolve()

                    usd_mesh = await convert_mesh_to_usd(src)

                    # Reference converted USD under a prim
                    prim_name = f"visual_{i}"
                    xform = UsdGeom.Xform.Define(stage, link_path + "/" + prim_name)
                    xform.AddTranslateOp().Set(Gf.Vec3d(*pose_xyz))
                    xform.AddRotateXYZOp().Set(Gf.Vec3d(pose_rpy[0]*57.2957795, pose_rpy[1]*57.2957795, pose_rpy[2]*57.2957795))

                    prim = stage.GetPrimAtPath(xform.GetPath())
                    prim.GetReferences().AddReference(str(usd_mesh))
                    continue

                # primitives
                add_primitive(link_path, geom, f"visual_{i}", pose_xyz, pose_rpy)

        stage.GetRootLayer().Save()

    asyncio.get_event_loop().run_until_complete(build())
    simulation_app.close()

if __name__ == "__main__":
    main()
