#!/usr/bin/env python

import copy
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