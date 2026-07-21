#!/usr/bin/env python

import copy
import math
import os
import sys
import xml.etree.ElementTree as ET
from xml.dom import minidom

import yaml


REFERENCE_ATTRS = (
    "body",
    "body1",
    "body2",
    "geom",
    "geom1",
    "geom2",
    "joint",
    "joint1",
    "joint2",
    "material",
    "mesh",
    "site",
    "site1",
    "site2",
    "target",
    "texture",
)

FILE_ATTR_TAGS = (
    "hfield",
    "mesh",
    "skin",
    "texture",
)


def add_xyz(lhs, rhs):
    lhs_values = [float(value) for value in lhs.split()]
    rhs_values = [float(value) for value in rhs]
    return "{} {} {}".format(
        lhs_values[0] + rhs_values[0],
        lhs_values[1] + rhs_values[1],
        lhs_values[2] + rhs_values[2],
    )


def prefix_value(value, prefix):
    if not value:
        return value
    return "{}_{}".format(prefix, value)


def prefix_tree(elem, prefix):
    if "name" in elem.attrib:
        elem.set("name", prefix_value(elem.attrib["name"], prefix))

    for attr in REFERENCE_ATTRS:
        if attr in elem.attrib:
            elem.set(attr, prefix_value(elem.attrib[attr], prefix))

    for child in elem:
        prefix_tree(child, prefix)


def rewrite_file_paths(elem, source_dir, output_dir):
    if elem.tag in FILE_ATTR_TAGS and "file" in elem.attrib:
        file_path = elem.attrib["file"]
        if not os.path.isabs(file_path):
            abs_file_path = os.path.normpath(os.path.join(source_dir, file_path))
            elem.set("file", os.path.relpath(abs_file_path, output_dir))

    if elem.tag == "include" and "file" in elem.attrib:
        include_path = elem.attrib["file"]
        if not os.path.isabs(include_path):
            abs_include_path = os.path.normpath(os.path.join(source_dir, include_path))
            elem.set("file", os.path.relpath(abs_include_path, output_dir))

    for child in elem:
        rewrite_file_paths(child, source_dir, output_dir)


def get_or_create(parent, tag):
    elem = parent.find(tag)
    if elem is None:
        elem = ET.SubElement(parent, tag)
    return elem


def prettify(root):
    xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
    return "\n".join(line for line in xmlstr.splitlines() if line.strip()) + "\n"


def format_numbers(values):
    return " ".join(str(value) for value in values)


def add_scene_objects(objects, asset_root, worldbody_root):
    """Add free-moving task objects declared in the scene YAML."""
    object_names = set()
    for index, object_config in enumerate(objects):
        if not isinstance(object_config, dict):
            raise ValueError("each objects entry must be a mapping")

        name = str(object_config.get("name", "object_{}".format(index + 1)))
        if name in object_names:
            raise ValueError("duplicate scene object name '{}'".format(name))
        object_names.add(name)

        object_type = object_config.get("type", "triangular_prism")
        if object_type == "cylinder":
            size = object_config.get("size", [0.09, 0.65])
            if len(size) != 2:
                raise ValueError("cylinder size must be [radius, height]")
            radius = float(size[0])
            height = float(size[1])
            if radius <= 0.0 or height <= 0.0:
                raise ValueError("cylinder dimensions must be positive")

            position = object_config.get("pos", [0.0, 0.0, height / 2.0])
            if len(position) != 3:
                raise ValueError("scene object pos must contain three values")

            body = ET.SubElement(worldbody_root, "body")
            body.set("name", name)
            body.set("pos", format_numbers(position))

            geom = ET.SubElement(body, "geom")
            geom.set("name", name + "_geom")
            geom.set("type", "cylinder")
            geom.set("size", format_numbers([radius, height / 2.0]))
            geom.set("contype", "1")
            geom.set("conaffinity", "1")
            geom.set("condim", "6")
            geom.set("friction", format_numbers(object_config.get("friction", [1.2, 0.02, 0.001])))
            geom.set("solref", format_numbers(object_config.get("solref", [0.01, 1.0])))
            geom.set("solimp", format_numbers(object_config.get("solimp", [0.9, 0.95, 0.003])))
            geom.set("rgba", format_numbers(object_config.get("rgba", [0.35, 0.38, 0.42, 1.0])))
            continue

        if object_type != "triangular_prism":
            raise ValueError("unsupported scene object type '{}'".format(object_type))

        size = object_config.get("size", [0.36, 0.30])
        if len(size) != 2:
            raise ValueError("triangular_prism size must be [triangle_side, height]")
        triangle_side = float(size[0])
        height = float(size[1])
        if triangle_side <= 0.0 or height <= 0.0:
            raise ValueError("triangular_prism dimensions must be positive")

        position = object_config.get("pos", [0.0, 0.0, 0.0])
        if len(position) != 3:
            raise ValueError("scene object pos must contain three values")
        mass = float(object_config.get("mass", 0.50))
        if mass <= 0.0:
            raise ValueError("scene object mass must be positive")

        # The prism axis is Z: three robots can approach the three rectangular
        # side faces, then a fourth can approach the triangular bottom face.
        circumradius = triangle_side / math.sqrt(3.0)
        x_rear = -circumradius / 2.0
        y_side = triangle_side / 2.0
        z_min = -height / 2.0
        z_max = height / 2.0
        vertices = [
            circumradius, 0.0, z_min,
            x_rear, y_side, z_min,
            x_rear, -y_side, z_min,
            circumradius, 0.0, z_max,
            x_rear, y_side, z_max,
            x_rear, -y_side, z_max,
        ]
        faces = [
            0, 2, 1,
            3, 4, 5,
            0, 1, 4, 0, 4, 3,
            1, 2, 5, 1, 5, 4,
            2, 0, 3, 2, 3, 5,
        ]

        mesh_name = name + "_mesh"
        mesh = ET.SubElement(asset_root, "mesh")
        mesh.set("name", mesh_name)
        mesh.set("vertex", format_numbers(vertices))
        mesh.set("face", format_numbers(faces))

        body = ET.SubElement(worldbody_root, "body")
        body.set("name", name)
        body.set("pos", format_numbers(position))
        if "quat" in object_config:
            quat = object_config["quat"]
            if len(quat) != 4:
                raise ValueError("scene object quat must contain four values")
            body.set("quat", format_numbers(quat))
        elif "euler" in object_config:
            euler = object_config["euler"]
            if len(euler) != 3:
                raise ValueError("scene object euler must contain three values")
            body.set("euler", format_numbers(euler))

        # Use the closed-form centroidal inertia instead of relying on signed
        # mesh-volume inference.  This is both exact for a uniform triangular
        # prism and avoids sensitivity to face winding in MuJoCo 2.3.x.
        horizontal_inertia = mass * (triangle_side * triangle_side / 24.0 +
                                     height * height / 12.0)
        vertical_inertia = mass * triangle_side * triangle_side / 12.0
        inertial = ET.SubElement(body, "inertial")
        inertial.set("pos", "0 0 0")
        inertial.set("mass", str(mass))
        inertial.set("diaginertia", format_numbers([
            horizontal_inertia,
            horizontal_inertia,
            vertical_inertia,
        ]))

        if object_config.get("free", True):
            free_joint = ET.SubElement(body, "freejoint")
            free_joint.set("name", name + "_root")

        geom = ET.SubElement(body, "geom")
        geom.set("name", name + "_geom")
        geom.set("type", "mesh")
        geom.set("mesh", mesh_name)
        geom.set("condim", "6")
        geom.set("friction", format_numbers(object_config.get("friction", [1.2, 0.02, 0.001])))
        geom.set("solref", format_numbers(object_config.get("solref", [0.01, 1.0])))
        geom.set("solimp", format_numbers(object_config.get("solimp", [0.9, 0.95, 0.003])))
        geom.set("rgba", format_numbers(object_config.get("rgba", [0.92, 0.45, 0.08, 1.0])))

        centre_site = ET.SubElement(body, "site")
        centre_site.set("name", name + "_centre")
        centre_site.set("size", "0.005")
        centre_site.set("rgba", "0 0 0 0")


def compose_scene(config_path):
    with open(config_path) as file_handle:
        config = yaml.safe_load(file_handle)

    config_dir = os.path.dirname(os.path.abspath(config_path))
    source_model = config["source_model"]
    output_model = config["output_model"]
    if not os.path.isabs(source_model):
        source_model = os.path.normpath(os.path.join(config_dir, source_model))
    if not os.path.isabs(output_model):
        output_model = os.path.normpath(os.path.join(config_dir, output_model))
    robots = config["robots"]

    source_tree = ET.parse(source_model)
    source_root = source_tree.getroot()
    source_dir = os.path.dirname(source_model)
    output_dir = os.path.dirname(output_model)

    scene_root = ET.Element(source_root.tag, source_root.attrib)
    if "model" in scene_root.attrib:
        scene_root.set("model", config.get("scene_name", scene_root.attrib["model"] + "_scene"))

    passthrough_tags = (
        "compiler",
        "option",
        "default",
        "size",
        "visual",
        "statistic",
        "extension",
        "custom",
    )
    for child in source_root:
        if child.tag in passthrough_tags:
            scene_root.append(copy.deepcopy(child))

    # MuJoCo 2.3.x switches from dense to sparse Jacobians at nv=60 when set
    # to auto.  Scenes with four compliant-foot Bees plus a free payload cross
    # that boundary and require the dense path for stable passive dynamics.
    if config.get("jacobian") is not None:
        option_root = get_or_create(scene_root, "option")
        option_root.set("jacobian", str(config["jacobian"]))

    asset_root = get_or_create(scene_root, "asset")
    worldbody_root = get_or_create(scene_root, "worldbody")
    actuator_root = get_or_create(scene_root, "actuator")
    sensor_root = get_or_create(scene_root, "sensor")

    source_asset = source_root.find("asset")
    source_worldbody = source_root.find("worldbody")
    source_actuator = source_root.find("actuator")
    source_sensor = source_root.find("sensor")

    for robot in robots:
        prefix = robot["name"]
        offset = robot.get("pos", [0.0, 0.0, 0.0])
        quat = robot.get("quat")

        if source_asset is not None:
            for child in source_asset:
                asset_child = copy.deepcopy(child)
                prefix_tree(asset_child, prefix)
                rewrite_file_paths(asset_child, source_dir, output_dir)
                asset_root.append(asset_child)

        if source_worldbody is not None:
            for child in source_worldbody:
                world_child = copy.deepcopy(child)
                prefix_tree(world_child, prefix)
                if world_child.tag == "body":
                    world_child.set("pos", add_xyz(world_child.attrib.get("pos", "0 0 0"), offset))
                    if quat is not None:
                        world_child.set("quat", "{} {} {} {}".format(quat[0], quat[1], quat[2], quat[3]))
                worldbody_root.append(world_child)

        if source_actuator is not None:
            for child in source_actuator:
                actuator_child = copy.deepcopy(child)
                prefix_tree(actuator_child, prefix)
                actuator_root.append(actuator_child)

        if source_sensor is not None:
            for child in source_sensor:
                sensor_child = copy.deepcopy(child)
                prefix_tree(sensor_child, prefix)
                sensor_root.append(sensor_child)

    add_scene_objects(config.get("objects", []), asset_root, worldbody_root)

    include_paths = set()
    for include_elem in source_root.findall("include"):
        include_path = include_elem.attrib.get("file")
        if include_path in include_paths:
            continue
        include_paths.add(include_path)
        include_child = copy.deepcopy(include_elem)
        rewrite_file_paths(include_child, source_dir, output_dir)
        scene_root.append(include_child)

    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    with open(output_model, "w") as file_handle:
        file_handle.write(prettify(scene_root))


def main():
    if len(sys.argv) != 2:
        print("Usage: rosrun mujoco_ros_control mujoco_scene_composer.py /absolute/path/to/scene.yaml")
        return 1

    compose_scene(sys.argv[1])
    return 0


if __name__ == "__main__":
    sys.exit(main())
