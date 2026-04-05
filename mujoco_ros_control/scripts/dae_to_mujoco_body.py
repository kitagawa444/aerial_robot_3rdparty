#!/usr/bin/env python
"""
Convert a DAE file to a MuJoCo XML body with colors/textures.

Usage:
  python dae_to_mujoco_body.py input.dae [output_dir] [--body-name NAME] [--pos "x y z"] [--euler "r p y"]

Generates in output_dir:
  - OBJ mesh file(s)
  - Texture PNG file(s) if present
  - body.xml  -- MuJoCo XML with <asset> and <worldbody><body> that can be <include>d

Example:
  python dae_to_mujoco_body.py model.dae ./mujoco_output --body-name obstacle --pos "1 0 0.5"

  Then in your main MuJoCo model:
    <include file="mujoco_output/body.xml"/>
"""

import os
import sys
import argparse
import xml.etree.ElementTree as ET
from xml.dom import minidom

try:
    import trimesh
except ImportError:
    print("Error: trimesh is required. Install with: pip install trimesh pycollada")
    sys.exit(1)

MAX_FACES = 199999


# ---------------------------------------------------------------------------
# Mesh helpers (reused from convert.py)
# ---------------------------------------------------------------------------

def try_decimate(mesh, target_faces):
    """Try to decimate mesh. Returns original mesh if not possible."""
    if len(mesh.faces) <= target_faces:
        return mesh
    try:
        return mesh.simplify_quadric_decimation(target_faces)
    except Exception:
        print("  Warning: mesh decimation not available, keeping original ({} faces)".format(len(mesh.faces)))
        return mesh


def get_mesh_color(mesh):
    """Extract diffuse color from mesh visual/material as 'r g b a' string."""
    try:
        if hasattr(mesh.visual, 'material'):
            mat = mesh.visual.material
            if hasattr(mat, 'main_color') and mat.main_color is not None:
                c = mat.main_color
                return "{} {} {} {}".format(
                    round(c[0] / 255.0, 6),
                    round(c[1] / 255.0, 6),
                    round(c[2] / 255.0, 6),
                    round(c[3] / 255.0, 6))
    except Exception:
        pass
    return None


def get_mesh_texture(mesh):
    """Extract texture image from mesh visual if it has one."""
    try:
        if hasattr(mesh.visual, 'material'):
            mat = mesh.visual.material
            if hasattr(mat, 'image') and mat.image is not None:
                return mat.image
            if hasattr(mat, 'baseColorTexture') and mat.baseColorTexture is not None:
                return mat.baseColorTexture
        if hasattr(mesh.visual, 'to_texture') and callable(mesh.visual.to_texture):
            tex_visual = mesh.visual.to_texture()
            if hasattr(tex_visual, 'material') and hasattr(tex_visual.material, 'image'):
                if tex_visual.material.image is not None:
                    return tex_visual.material.image
    except Exception:
        pass
    return None


def get_geometries_from_scene(scene):
    """Extract (name, mesh, color, texture_image) list from a trimesh Scene."""
    results = []
    try:
        for node_name in scene.graph.nodes_geometry:
            transform, geometry_name = scene.graph[node_name]
            if geometry_name in scene.geometry:
                mesh = scene.geometry[geometry_name].copy()
                mesh.apply_transform(transform)
                color = get_mesh_color(mesh)
                texture = get_mesh_texture(mesh)
                results.append((geometry_name, mesh, color, texture))
    except Exception as e:
        print("  Warning: scene graph error ({}), using raw geometries".format(e))
        for geom_name, mesh in scene.geometry.items():
            color = get_mesh_color(mesh)
            texture = get_mesh_texture(mesh)
            results.append((geom_name, mesh.copy(), color, texture))
    return results


def save_texture_image(texture_image, output_dir, base_name):
    """Save a PIL Image texture to disk. Returns the filename or None."""
    if texture_image is None:
        return None
    try:
        tex_filename = base_name + ".png"
        tex_path = os.path.join(output_dir, tex_filename)
        texture_image.save(tex_path)
        return tex_filename
    except Exception as e:
        print("  Warning: failed to save texture: {}".format(e))
        return None


# ---------------------------------------------------------------------------
# DAE -> sub-meshes
# ---------------------------------------------------------------------------

def extract_sub_meshes(input_file, output_dir, max_faces=MAX_FACES):
    """
    Load a DAE file with trimesh, export OBJ sub-meshes to output_dir.
    Returns a list of dicts: [{"name", "file", "color", "texture"}, ...]
    """
    base_name = os.path.splitext(os.path.basename(input_file))[0]
    sub_meshes = []

    print("Processing: {}".format(input_file))

    try:
        loaded = trimesh.load(input_file)
    except Exception as e:
        print("  Error loading: {}".format(e))
        return sub_meshes

    if isinstance(loaded, trimesh.Trimesh):
        mesh = try_decimate(loaded, max_faces)
        obj_file = base_name + ".obj"
        mesh.export(os.path.join(output_dir, obj_file))
        texture = get_mesh_texture(mesh)
        tex_file = save_texture_image(texture, output_dir, base_name + "_tex")
        sub_meshes.append({
            "name": base_name,
            "file": obj_file,
            "color": get_mesh_color(mesh),
            "texture": tex_file,
        })

    elif isinstance(loaded, trimesh.Scene):
        geometries = get_geometries_from_scene(loaded)

        if len(geometries) == 0:
            print("  Warning: no geometry found")
            return sub_meshes

        if len(geometries) == 1:
            geom_name, mesh, color, texture = geometries[0]
            mesh = try_decimate(mesh, max_faces)
            obj_file = base_name + ".obj"
            mesh.export(os.path.join(output_dir, obj_file))
            tex_file = save_texture_image(texture, output_dir, base_name + "_tex")
            sub_meshes.append({
                "name": base_name,
                "file": obj_file,
                "color": color,
                "texture": tex_file,
            })
        else:
            for i, (geom_name, mesh, color, texture) in enumerate(geometries):
                mesh = try_decimate(mesh, max_faces)
                sub_name = "{}_{}".format(base_name, i)
                obj_file = sub_name + ".obj"
                mesh.export(os.path.join(output_dir, obj_file))
                tex_file = save_texture_image(texture, output_dir, sub_name + "_tex")
                sub_meshes.append({
                    "name": sub_name,
                    "file": obj_file,
                    "color": color,
                    "texture": tex_file,
                })

    print("  -> {} sub-mesh(es)".format(len(sub_meshes)))
    return sub_meshes


# ---------------------------------------------------------------------------
# MuJoCo XML generation
# ---------------------------------------------------------------------------

def generate_mujoco_body_xml(sub_meshes, output_dir, body_name, body_pos, body_euler):
    """
    Generate a MuJoCo XML file that can be <include>d into another model.

    The file contains:
      <mujoco>
        <asset>  ... mesh / texture / material definitions ...  </asset>
        <worldbody>
          <body name="..." pos="...">
            <geom ... />   (one per sub-mesh)
          </body>
        </worldbody>
      </mujoco>
    """
    mujoco = ET.Element("mujoco")

    # --- asset ---
    asset = ET.SubElement(mujoco, "asset")

    for sub in sub_meshes:
        # mesh
        mesh_elem = ET.SubElement(asset, "mesh")
        mesh_elem.set("name", sub["name"])
        mesh_elem.set("file", sub["file"])

        # texture (if any)
        if sub.get("texture"):
            tex_name = "tex_" + sub["name"]
            tex_elem = ET.SubElement(asset, "texture")
            tex_elem.set("name", tex_name)
            tex_elem.set("file", sub["texture"])
            tex_elem.set("type", "2d")

            mat_name = "mat_" + sub["name"]
            mat_elem = ET.SubElement(asset, "material")
            mat_elem.set("name", mat_name)
            mat_elem.set("texture", tex_name)

        # color-only material (if no texture but color exists)
        elif sub.get("color"):
            mat_name = "mat_" + sub["name"]
            mat_elem = ET.SubElement(asset, "material")
            mat_elem.set("name", mat_name)
            mat_elem.set("rgba", sub["color"])

    # --- worldbody -> body ---
    worldbody = ET.SubElement(mujoco, "worldbody")
    body = ET.SubElement(worldbody, "body")
    body.set("name", body_name)
    body.set("pos", body_pos)
    if body_euler:
        body.set("euler", body_euler)

    for sub in sub_meshes:
        geom = ET.SubElement(body, "geom")
        geom.set("type", "mesh")
        geom.set("mesh", sub["name"])
        geom.set("contype", "1")
        geom.set("conaffinity", "1")

        # apply material if we created one
        if sub.get("texture") or sub.get("color"):
            geom.set("material", "mat_" + sub["name"])

    # --- write XML ---
    xml_str = minidom.parseString(ET.tostring(mujoco)).toprettyxml(indent="  ")
    # remove blank lines
    lines = [line for line in xml_str.split("\n") if line.strip()]
    xml_str = "\n".join(lines) + "\n"

    output_path = os.path.join(output_dir, "body.xml")
    with open(output_path, "w") as f:
        f.write(xml_str)

    print("Generated: {}".format(output_path))
    return output_path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Convert a DAE file to a MuJoCo XML body with colors/textures."
    )
    parser.add_argument("input_dae", help="Path to the input .dae file")
    parser.add_argument("output_dir", nargs="?", default=None,
                        help="Output directory (default: same directory as input, under <name>_mujoco/)")
    parser.add_argument("--body-name", default=None,
                        help="Name for the <body> element (default: DAE filename without extension)")
    parser.add_argument("--pos", default="0 0 0",
                        help='Position of the body, e.g. "1 0 0.5" (default: "0 0 0")')
    parser.add_argument("--euler", default=None,
                        help='Euler angles (radian) of the body, e.g. "0 0 1.57"')
    parser.add_argument("--max-faces", type=int, default=MAX_FACES,
                        help="Maximum number of faces per sub-mesh (default: {})".format(MAX_FACES))

    args = parser.parse_args()

    max_faces = args.max_faces

    input_dae = os.path.abspath(args.input_dae)
    if not os.path.isfile(input_dae):
        print("Error: file not found: {}".format(input_dae))
        sys.exit(1)

    base_name = os.path.splitext(os.path.basename(input_dae))[0]

    if args.output_dir:
        output_dir = os.path.abspath(args.output_dir)
    else:
        output_dir = os.path.join(os.path.dirname(input_dae), base_name + "_mujoco")

    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    body_name = args.body_name if args.body_name else base_name

    # 1. Convert DAE -> OBJ sub-meshes
    sub_meshes = extract_sub_meshes(input_dae, output_dir, max_faces)
    if not sub_meshes:
        print("Error: no meshes extracted from {}".format(input_dae))
        sys.exit(1)

    # 2. Generate MuJoCo XML
    xml_path = generate_mujoco_body_xml(
        sub_meshes, output_dir, body_name, args.pos, args.euler
    )

    print("\nDone! To use in your MuJoCo model, add:")
    print('  <include file="{}"/>'.format(os.path.relpath(xml_path)))
    print("\nThe body '{}' will appear at pos=\"{}\"".format(body_name, args.pos))


if __name__ == "__main__":
    main()
