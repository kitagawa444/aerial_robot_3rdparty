import os
import sys
import json
import shutil

try:
    import trimesh
except ImportError:
    print("Error: trimesh is required. Install with: pip install trimesh pycollada")
    sys.exit(1)

meshdir = sys.argv
print(len(meshdir))
if len(meshdir) != 2:
    print("$ python convert.py meshdir")
    sys.exit()

meshdir = meshdir[1]

MAX_FACES = 199999


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
    """Extract texture image file path from mesh visual if it has one."""
    try:
        if hasattr(mesh.visual, 'material'):
            mat = mesh.visual.material
            if hasattr(mat, 'image') and mat.image is not None:
                # PBR material with image
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
    """Extract (name, mesh_with_transform, color, texture_image) list from a trimesh Scene."""
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


def process_dae(input_file, output_dir):
    """Convert a DAE to OBJ(s), splitting by material. Writes a meta JSON."""
    base_name = os.path.splitext(os.path.basename(input_file))[0]
    meta = {"sub_meshes": []}

    print("Processing: {}".format(input_file))

    try:
        loaded = trimesh.load(input_file)
    except Exception as e:
        print("  Error loading: {}".format(e))
        return

    if isinstance(loaded, trimesh.Trimesh):
        mesh = try_decimate(loaded, MAX_FACES)
        obj_file = base_name + ".obj"
        mesh.export(os.path.join(output_dir, obj_file))
        texture = get_mesh_texture(mesh)
        tex_file = save_texture_image(texture, output_dir, base_name + "_tex")
        meta["sub_meshes"].append({
            "name": base_name,
            "file": obj_file,
            "color": get_mesh_color(mesh),
            "texture": tex_file
        })

    elif isinstance(loaded, trimesh.Scene):
        geometries = get_geometries_from_scene(loaded)

        if len(geometries) == 0:
            print("  Warning: no geometry found")
            return

        if len(geometries) == 1:
            geom_name, mesh, color, texture = geometries[0]
            mesh = try_decimate(mesh, MAX_FACES)
            obj_file = base_name + ".obj"
            mesh.export(os.path.join(output_dir, obj_file))
            tex_file = save_texture_image(texture, output_dir, base_name + "_tex")
            meta["sub_meshes"].append({
                "name": base_name,
                "file": obj_file,
                "color": color,
                "texture": tex_file
            })
        else:
            for i, (geom_name, mesh, color, texture) in enumerate(geometries):
                mesh = try_decimate(mesh, MAX_FACES)
                sub_name = "{}_{}".format(base_name, i)
                obj_file = sub_name + ".obj"
                mesh.export(os.path.join(output_dir, obj_file))
                tex_file = save_texture_image(texture, output_dir, sub_name + "_tex")
                meta["sub_meshes"].append({
                    "name": sub_name,
                    "file": obj_file,
                    "color": color,
                    "texture": tex_file
                })

    meta_path = os.path.join(output_dir, base_name + "_meta.json")
    with open(meta_path, 'w') as f:
        json.dump(meta, f, indent=2)
    print("  -> {} sub-mesh(es)".format(len(meta["sub_meshes"])))


def process_subdirectories(root):
    for foldername, subfolders, filenames in os.walk(root):
        for filename in filenames:
            if filename.endswith(".dae"):
                input_file = os.path.join(foldername, filename)
                process_dae(input_file, foldername)


process_subdirectories(meshdir)
