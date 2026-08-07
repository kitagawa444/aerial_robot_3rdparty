#!/usr/bin/env python

import xml.etree.ElementTree as ET
from xml.dom import minidom
import subprocess
import yaml
import rospkg
import os
import sys
import rospy
import shutil
import json
import copy
import math

rotor_list = []
joint_list = []
mesh_texture_map = {}  # mesh_name -> texture_filename
mesh_color_map = {}  # mesh_name -> "r g b a" string from DAE diffuse color
rospack = rospkg.RosPack()


def run_subprocess(cmd):
    if sys.version.split(".")[0] == "2":
        subprocess.call(cmd, shell=True)
    if sys.version.split(".")[0] == "3":
        subprocess.run(cmd, shell=True)


def get_filename(filepath):
    return filepath.rsplit("/", 1)[1]


def get_directory(filepath):
    return filepath.rsplit("/", 1)[0]


def remove_extension(filename):
    before_ext, ext = os.path.splitext(filename)
    return before_ext


def run_xacro(input_path, output_path):
    cmd = "rosrun xacro xacro {} > {}".format(input_path, output_path)
    run_subprocess(cmd)


def process_urdf(package, urdf_path, workdir_path):
    global rotor_list
    global joint_list
    global mesh_texture_map
    global mesh_color_map
    rotor_list = []
    joint_list = []
    mesh_texture_map = {}
    mesh_color_map = {}
    urdf_tree = ET.parse(urdf_path)
    urdf_root = urdf_tree.getroot()

    # add mujoco config
    mujoco = ET.Element('mujoco')
    compiler = ET.SubElement(mujoco, 'compiler')
    option = ET.SubElement(mujoco, 'option')
    compiler.set('balanceinertia', 'true')
    urdf_root.append(mujoco)

    # fix mesh path in visual tag
    for link in urdf_root.findall("link"):
        visuals_to_remove = []
        visuals_to_add = []

        for link_visual in link.findall("visual"):
            for link_visual_geometry in link_visual.findall("geometry"):
                for link_visual_geometry_mesh in link_visual_geometry.findall("mesh"):
                    filepath = link_visual_geometry_mesh.attrib["filename"]
                    search_string = "package://"
                    index = filepath.find(search_string)
                    filename = filepath[index + len(search_string):]
                    filename = filename[filename.find("/"):]
                    filepath = rospack.get_path(package) + filename

                    # get base path and mesh directory
                    base_path, ex = os.path.splitext(filepath)
                    mesh_dir = get_directory(base_path)
                    mesh_base = get_filename(base_path)

                    # check for meta JSON from convert.py
                    meta_path = os.path.join(mesh_dir, mesh_base + "_meta.json")

                    if os.path.isfile(meta_path):
                        with open(meta_path) as mf:
                            meta = json.load(mf)
                        sub_meshes = meta["sub_meshes"]

                        # copy all sub-mesh files to workdir and register colors/textures
                        for sub in sub_meshes:
                            obj_src = os.path.join(mesh_dir, sub["file"])
                            if os.path.isfile(obj_src):
                                shutil.copy(obj_src, workdir_path)
                            # handle texture
                            if sub.get("texture"):
                                tex_src = os.path.join(mesh_dir, sub["texture"])
                                if os.path.isfile(tex_src):
                                    shutil.copy(tex_src, workdir_path)
                                    mesh_texture_map[sub["name"]] = sub["texture"]
                            # handle color (only if no texture)
                            if sub.get("color") and sub["name"] not in mesh_texture_map:
                                mesh_color_map[sub["name"]] = sub["color"]

                        if len(sub_meshes) == 1:
                            # single sub-mesh: update geometry in place
                            geometry_elem = ET.Element('geometry')
                            mesh_elem = ET.Element("mesh")
                            mesh_elem.set("filename", sub_meshes[0]["file"])
                            geometry_elem.append(mesh_elem)
                            link_visual.remove(link_visual_geometry)
                            link_visual.append(geometry_elem)
                        else:
                            # multiple sub-meshes: split into multiple visuals
                            origin = link_visual.find("origin")
                            visuals_to_remove.append(link_visual)
                            for sub in sub_meshes:
                                new_visual = ET.Element("visual")
                                if origin is not None:
                                    new_visual.append(copy.deepcopy(origin))
                                new_geometry = ET.Element("geometry")
                                new_mesh = ET.Element("mesh")
                                new_mesh.set("filename", sub["file"])
                                new_geometry.append(new_mesh)
                                new_visual.append(new_geometry)
                                visuals_to_add.append(new_visual)
                    else:
                        # fallback: no meta JSON, try direct OBJ
                        filepath = base_path + ".obj"
                        filename = get_filename(filepath)
                        if os.path.isfile(filepath):
                            shutil.copy(filepath, workdir_path)
                        geometry_elem = ET.Element('geometry')
                        mesh_elem = ET.Element("mesh")
                        mesh_elem.set("filename", filename)
                        geometry_elem.append(mesh_elem)
                        link_visual.remove(link_visual_geometry)
                        link_visual.append(geometry_elem)

        # apply deferred visual modifications
        for v in visuals_to_remove:
            link.remove(v)
        for v in visuals_to_add:
            link.append(v)

    # replace collision tag by mesh
    ## remove initial collision
    for link in urdf_root.findall("link"):
        for link_collision in link.findall("collision"):
            link.remove(link_collision)

    ## copy from visual (one collision per visual)
    for link in urdf_root.findall("link"):
        link_name = link.attrib["name"]
        for i, link_visual in enumerate(link.findall("visual")):
            collision_tag = ET.Element("collision")
            if i == 0:
                collision_tag.set("name", link_name)
            else:
                collision_tag.set("name", "{}_{}".format(link_name, i))
            for link_visual_elem in link_visual:
                collision_tag.append(copy.deepcopy(link_visual_elem))
            link.append(collision_tag)

    # get actuator list
    for transmission in urdf_root.findall("transmission"):
        for transmission_joint in transmission.findall("joint"):
            name = transmission_joint.attrib["name"]
            for transmission_joint_hardwareinterface in transmission_joint.findall("hardwareInterface"):
                if transmission_joint_hardwareinterface.text == "RotorInterface":
                    rotor_list.append(name)
                if transmission_joint_hardwareinterface.text == "hardware_interface/EffortJointInterface":
                    joint_list.append(name)
        urdf_root.remove(transmission)

    # output modified urdf
    xmlstr = minidom.parseString(ET.tostring(urdf_root)).toprettyxml(indent="  ")
    with open(urdf_path, 'w') as f:
        f.write(xmlstr)

    # remove blank lines in urdf
    cmd = "sed -i '/^[[:space:]]*$/d' {}".format(urdf_path)
    run_subprocess(cmd)


def generate_xml(urdf_path, mujoco_path):
    # compile by mujoco
    mujoco_ros_control = rospack.get_path("mujoco_ros_control")
    mujoco_compile_path = os.path.join(mujoco_ros_control, "build/mujoco-2.3.7/bin/compile")
    cmd = "{} {} {}".format(mujoco_compile_path, urdf_path, mujoco_path)
    run_subprocess(cmd)

def format_numbers(values):
    return " ".join(str(value) for value in values)


def add_compliant_feet(mujoco_root, feet_config):
    """Add an optional array of spring-loaded spherical contact pads."""
    if not feet_config:
        return

    parent_body_name = feet_config.get("parent_body", "base_link")
    parent_body = None
    for body in mujoco_root.iter("body"):
        if body.attrib.get("name") == parent_body_name:
            parent_body = body
            break
    if parent_body is None:
        raise ValueError("compliant_feet parent body '{}' was not found".format(parent_body_name))

    contact_prefix = str(feet_config.get("name_prefix", "spring_foot"))
    if not contact_prefix:
        raise ValueError("compliant contact name_prefix must not be empty")

    feet = feet_config.get("feet", [])
    if not feet:
        raise ValueError("compliant_feet.feet must contain at least one foot")

    axis = [float(value) for value in feet_config.get("axis", [0.0, 0.0, 1.0])]
    if len(axis) != 3:
        raise ValueError("compliant_feet.axis must contain three values")
    axis_norm = math.sqrt(sum(value * value for value in axis))
    if axis_norm <= 0.0:
        raise ValueError("compliant_feet.axis must be non-zero")
    axis = [value / axis_norm for value in axis]

    joint_range = feet_config.get("joint_range", [-0.001, 0.015])
    if len(joint_range) != 2 or float(joint_range[0]) >= float(joint_range[1]):
        raise ValueError("compliant_feet.joint_range must be [lower, upper]")

    stiffness = float(feet_config.get("stiffness", 600.0))
    damping = float(feet_config.get("damping", 3.0))
    spring_reference = float(feet_config.get("spring_reference", 0.0))
    free_length = float(feet_config.get("free_length", 0.024))
    shaft_radius = float(feet_config.get("shaft_radius", 0.003))
    shaft_mass = float(feet_config.get("shaft_mass", 0.001))
    ball_radius = float(feet_config.get("ball_radius", 0.012))
    ball_mass = float(feet_config.get("ball_mass", 0.006))
    friction = feet_config.get("friction", [1.2, 0.02, 0.001])
    solref = feet_config.get("solref", [0.01, 1.0])
    solimp = feet_config.get("solimp", [0.9, 0.95, 0.003])
    ball_rgba = feet_config.get("ball_rgba", [0.04, 0.04, 0.04, 1.0])
    shaft_rgba = feet_config.get("shaft_rgba", [0.65, 0.65, 0.65, 1.0])

    ball_offset = [-free_length * value for value in axis]
    shaft_fromto = [0.0, 0.0, 0.0] + ball_offset

    sensor_root = mujoco_root.find("sensor")
    if sensor_root is None:
        sensor_root = ET.SubElement(mujoco_root, "sensor")

    foot_names = set()
    for index, foot_config in enumerate(feet):
        if not isinstance(foot_config, dict):
            raise ValueError("each compliant_feet.feet entry must be a mapping")
        foot_name = str(foot_config.get("name", index + 1))
        if foot_name in foot_names:
            raise ValueError("duplicate compliant foot name '{}'".format(foot_name))
        foot_names.add(foot_name)

        position = foot_config.get("pos")
        if position is None or len(position) != 3:
            raise ValueError("compliant foot '{}' must have a three-value pos".format(foot_name))

        name_prefix = "{}_{}".format(contact_prefix, foot_name)
        foot_body = ET.SubElement(parent_body, "body")
        foot_body.set("name", name_prefix)
        foot_body.set("pos", format_numbers(position))

        foot_joint = ET.SubElement(foot_body, "joint")
        foot_joint.set("name", name_prefix + "_joint")
        foot_joint.set("type", "slide")
        foot_joint.set("axis", format_numbers(axis))
        foot_joint.set("limited", "true")
        foot_joint.set("range", format_numbers(joint_range))
        foot_joint.set("stiffness", str(stiffness))
        foot_joint.set("damping", str(damping))
        foot_joint.set("springref", str(spring_reference))

        shaft_geom = ET.SubElement(foot_body, "geom")
        shaft_geom.set("name", name_prefix + "_spring")
        shaft_geom.set("type", "capsule")
        shaft_geom.set("fromto", format_numbers(shaft_fromto))
        shaft_geom.set("size", str(shaft_radius))
        shaft_geom.set("mass", str(shaft_mass))
        shaft_geom.set("contype", "0")
        shaft_geom.set("conaffinity", "0")
        shaft_geom.set("rgba", format_numbers(shaft_rgba))

        ball_geom = ET.SubElement(foot_body, "geom")
        ball_geom.set("name", name_prefix + "_ball")
        ball_geom.set("type", "sphere")
        ball_geom.set("pos", format_numbers(ball_offset))
        ball_geom.set("size", str(ball_radius))
        ball_geom.set("mass", str(ball_mass))
        ball_geom.set("contype", "1")
        ball_geom.set("conaffinity", "1")
        ball_geom.set("condim", "6")
        ball_geom.set("friction", format_numbers(friction))
        ball_geom.set("solref", format_numbers(solref))
        ball_geom.set("solimp", format_numbers(solimp))
        ball_geom.set("rgba", format_numbers(ball_rgba))

        contact_site = ET.SubElement(foot_body, "site")
        contact_site.set("name", name_prefix + "_contact")
        contact_site.set("type", "sphere")
        contact_site.set("pos", format_numbers(ball_offset))
        contact_site.set("size", str(ball_radius * 1.05))
        contact_site.set("rgba", "0 0 0 0")

        touch_sensor = ET.SubElement(sensor_root, "touch")
        touch_sensor.set("name", name_prefix + "_touch")
        touch_sensor.set("site", name_prefix + "_contact")

        # An inline force sensor at the rubber ball centre gives the 3D
        # contact force for this foot independently of the other feet.  The
        # site belongs to the spring-loaded child body, so MuJoCo reports the
        # force transmitted between that body and base_link in site axes.
        force_sensor = ET.SubElement(sensor_root, "force")
        force_sensor.set("name", name_prefix + "_force")
        force_sensor.set("site", name_prefix + "_contact")

        position_sensor = ET.SubElement(sensor_root, "jointpos")
        position_sensor.set("name", name_prefix + "_compression")
        position_sensor.set("joint", name_prefix + "_joint")

        velocity_sensor = ET.SubElement(sensor_root, "jointvel")
        velocity_sensor.set("name", name_prefix + "_compression_velocity")
        velocity_sensor.set("joint", name_prefix + "_joint")


def process_xml(urdf_path, mujoco_path, model_config=None):
    mujoco_tree = ET.parse(mujoco_path)
    mujoco_root = mujoco_tree.getroot()
    urdf_tree = ET.parse(urdf_path)
    urdf_root = urdf_tree.getroot()

    # map of (child, parent)
    mujoco_parent_map = dict((c, p) for p in mujoco_tree.iter() for c in p)
    urdf_parent_map = dict((c, p) for p in urdf_tree.iter() for c in p)

    # get m_f_rate from urdf
    m_f_rate = 0.0
    for m_f_rate_elem in urdf_root.iter("m_f_rate"):
        m_f_rate = m_f_rate_elem.attrib["value"]

    compiler = mujoco_root.find("compiler")
    compiler.set("balanceinertia", "true")

    # process joints
    thrusts = ""
    rotor_axis_dict = {}
    for joint in mujoco_root.iter("joint"):
        ## for joint
        if "joint" in joint.attrib["name"]:
            joint.set("damping", "150") # modify damping
        ## for rotor
        if "rotor" in joint.attrib["name"]:
            ### get control range
            thrusts = joint.attrib["range"]

            ### get rotor axis for counter torque
            axis = joint.attrib["axis"].split()[2]
            rotor_axis_dict[joint.attrib["name"]] = axis

            ### add site at rotor
            parent_body = mujoco_parent_map[joint]
            site_elem = ET.Element("site")
            site_elem.set("name", joint.attrib["name"])
            site_elem.set("pos", "0 0 0")
            parent_body.remove(joint)
            parent_body.append(site_elem)

    # surround root link by body tag
    ## find root link
    root_link_name = ""
    for joint in urdf_root.iter("joint"):
        is_root_joint = False
        for joint_elem in joint:
            if joint_elem.tag == "parent" and joint_elem.attrib["link"] == "root":
                is_root_joint = True
        if is_root_joint:
            for joint_elem in joint:
                if joint_elem.tag == "child":
                    root_link_name = joint_elem.attrib["link"]

    ## get root link inertial
    root_link_mass = ""
    root_link_origin = ""
    root_link_inertia = ""
    for link in urdf_root.iter("link"):
        if link.attrib["name"] == root_link_name:
            for link_elem in link:
                if(link_elem.tag == "inertial"):
                    link_inertial = link_elem
                    for link_inertial_elem in link_inertial:
                        if link_inertial_elem.tag == "mass":
                            root_link_mass = link_inertial_elem.attrib["value"]
                        if link_inertial_elem.tag == "origin":
                            root_link_origin = link_inertial_elem.attrib["xyz"]
                        if link_inertial_elem.tag == "inertia":
                            root_link_inertia = link_inertial_elem.attrib["ixx"] + " " + link_inertial_elem.attrib["iyy"] + " " + link_inertial_elem.attrib["izz"] + " " + link_inertial_elem.attrib["ixy"] + " " + link_inertial_elem.attrib["ixz"] + " " + link_inertial_elem.attrib["iyz"]

    ## add root link body
    worldbody_elem_list = []
    for worldbody in mujoco_root.iter("worldbody"):
        for worldbody_elem in worldbody:
            worldbody_elem_list.append(worldbody_elem)
        for worldbody_elem in  worldbody_elem_list:
            worldbody.remove(worldbody_elem)

        root_link_elem = ET.Element("body")
        root_link_elem.set("name", root_link_name)
        root_link_elem.set("pos", "0 0 0.2")
        inertial_elem = ET.Element("inertial")
        inertial_elem.set("pos", root_link_origin)
        inertial_elem.set("mass", root_link_mass)
        inertial_elem.set("fullinertia", root_link_inertia)
        root_link_elem.append(inertial_elem)
        freejoint_elem = ET.Element("freejoint")
        root_link_elem.append(freejoint_elem)

        for worldbody_elem in worldbody_elem_list:
            root_link_elem.append(worldbody_elem)
        worldbody.append(root_link_elem)

    # add site at fc
    fc_name = ""
    fc_joint = ""
    fc_parent = ""
    fc_rel_pos = ""
    ## find fc name
    for baselink in urdf_root.iter("baselink"):
        fc_name = baselink.attrib["name"]
    ## find parent link
    for child in urdf_root.iter("child"):
        if child.attrib["link"] == fc_name:
            fc_joint = urdf_parent_map[child]
            for fc_joint_elem in fc_joint:
                if fc_joint_elem.tag == "parent":
                    fc_parent = fc_joint_elem.attrib["link"]
                if fc_joint_elem.tag == "origin":
                    fc_rel_pos = fc_joint_elem.attrib["xyz"]
    ## add site fc
    for mujoco_body in mujoco_root.iter("body"):
        if mujoco_body.attrib["name"] == fc_parent:
            site_elem = ET.Element("site")
            site_elem.set("name", fc_name)
            site_elem.set("pos", fc_rel_pos)
            mujoco_body.append(site_elem)


    # add inertial to fixed links connected to root link
    ## search urdf joint
    ### links conected to root link with fixed joint
    child_link_name_list = []
    #### get link names
    for urdf_joint in urdf_root.iter("joint"):
        if urdf_joint.attrib["type"] == "fixed":
            is_parent_root = False
            parent_link_name = ""
            child_link_name = ""
            child_link_origin_xyz = ""
            child_link_origin_rpy = ""
            for urdf_joint_elem in urdf_joint:
                if urdf_joint_elem.tag == "parent":
                    parent_link_name = urdf_joint_elem.attrib["link"]
                    if parent_link_name == root_link_name:
                        is_parent_root = True
                if urdf_joint_elem.tag == "child":
                    child_link_name = urdf_joint_elem.attrib["link"]
            if is_parent_root:
                child_link_name_list.append(child_link_name)


    #### search visual and inertial
    mesh_list = []
    mass_list = []
    origin_pos_list = []
    inertia_list = []
    link_list = []
    for child_link_name in child_link_name_list:
        for link in urdf_root.iter("link"):
            if link.attrib["name"] == child_link_name:
                is_exist_inertial = False
                is_exist_visual = False
                inertial_mass = ""
                inertial_origin_pos = ""
                inertial_inertia = ""
                for link_elem in link:
                    if link_elem.tag == "visual":
                        is_exist_visual = True
                    if link_elem.tag == "inertial":
                        is_exist_inertial = True
                        for link_inertial_elem in link_elem:
                            if link_inertial_elem.tag == "mass":
                                inertial_mass = link_inertial_elem.attrib["value"]
                            if link_inertial_elem.tag == "origin":
                                inertial_origin_pos = link_inertial_elem.attrib["xyz"]
                            if link_inertial_elem.tag == "inertia":
                                inertial_inertia = link_inertial_elem.attrib["ixx"] + " " + link_inertial_elem.attrib["iyy"] + " " + link_inertial_elem.attrib["izz"] + " " + link_inertial_elem.attrib["ixy"] + " " + link_inertial_elem.attrib["ixz"] + " " + link_inertial_elem.attrib["iyz"]

                if is_exist_inertial and is_exist_visual:
                    mass_list.append(inertial_mass)
                    origin_pos_list.append(inertial_origin_pos)
                    inertia_list.append(inertial_inertia)
                    link_list.append(link.attrib["name"])

    # output modified mujoco model
    xmlstr = minidom.parseString(ET.tostring(mujoco_root)).toprettyxml(indent="  ")
    with open(mujoco_path, "w") as f:
        f.write(xmlstr)

    # remove brank line in xml
    cmd = "sed -i '/^[[:space:]]*$/d' {}".format(mujoco_path)
    run_subprocess(cmd)

    # reload mujoco model
    mujoco_tree = ET.parse(mujoco_path)
    mujoco_root = mujoco_tree.getroot()
    mujoco_parent_map = dict((c, p) for p in mujoco_tree.iter() for c in p)

    ### modify xml
    for geom in mujoco_root.iter("geom"):
        if geom.attrib["name"] in link_list:
            geom_pos = "0 0 0"
            geom_quat = "1 0 0 0"
            if "pos" in geom.attrib:
                geom_pos = geom.attrib["pos"]
            if "quat" in geom.attrib:
                geom_quat = geom.attrib["quat"]
            parent_body = mujoco_parent_map[geom]
            index = link_list.index(geom.attrib["name"])
            body_elem = ET.Element("body")
            body_elem.set("name", link_list[index])
            body_elem.set("pos", geom_pos)
            body_elem.set("quat", geom_quat)
            inertial_elem = ET.Element("inertial")
            inertial_elem.set("pos", origin_pos_list[index])
            inertial_elem.set("mass", mass_list[index])
            inertial_elem.set("fullinertia", inertia_list[index])
            body_elem.append(inertial_elem)
            parent_body.append(body_elem)

    # process geoms
    for geom in mujoco_root.iter("geom"):
        geom.set("contype", "1")
        geom.set("conaffinity", "0")

    # add textures, materials, and colors
    global mesh_texture_map
    global mesh_color_map
    has_materials = mesh_texture_map or mesh_color_map
    if has_materials:
        asset = mujoco_root.find("asset")
        if asset is None:
            asset = ET.SubElement(mujoco_root, "asset")

        # texture-based materials
        for mesh_name, tex_file in mesh_texture_map.items():
            tex_name = "tex_" + mesh_name
            mat_name = "mat_" + mesh_name

            tex_elem = ET.SubElement(asset, "texture")
            tex_elem.set("name", tex_name)
            tex_elem.set("file", tex_file)
            tex_elem.set("type", "2d")

            mat_elem = ET.SubElement(asset, "material")
            mat_elem.set("name", mat_name)
            mat_elem.set("texture", tex_name)

        # color-based materials (from DAE diffuse color)
        for mesh_name, rgba in mesh_color_map.items():
            if mesh_name not in mesh_texture_map:
                mat_name = "mat_" + mesh_name
                mat_elem = ET.SubElement(asset, "material")
                mat_elem.set("name", mat_name)
                mat_elem.set("rgba", rgba)

        # apply materials to geoms
        all_material_meshes = set(mesh_texture_map.keys()) | set(mesh_color_map.keys())
        for geom in mujoco_root.iter("geom"):
            if "mesh" in geom.attrib:
                mesh_ref = geom.attrib["mesh"]
                if mesh_ref in all_material_meshes:
                    geom.set("material", "mat_" + mesh_ref)

    # A rotor URDF normally has a positive minimum controllable thrust, but
    # zero is also a distinct and valid command while the motors are stopped.
    # MuJoCo's ctrlrange clips zero to that positive lower limit, otherwise,
    # causing an unarmed vehicle to produce thrust on the ground.  Models that
    # need the stopped state can explicitly extend only the MuJoCo range to 0.
    rotor_ctrlrange = thrusts
    if model_config is not None and model_config.get("allow_zero_rotor_force", False):
        thrust_range = thrusts.split()
        if len(thrust_range) != 2:
            raise ValueError("rotor thrust range must contain lower and upper values")
        rotor_ctrlrange = format_numbers([0.0, float(thrust_range[1])])

    # actuators
    global rotor_list
    global joint_list
    actuator_elem = ET.Element("actuator")
    ## rotors
    for rotor in rotor_list:
        rotor_elem = ET.Element("motor")
        rotor_elem.set("name", rotor)
        rotor_elem.set("ctrllimited", "true")
        rotor_elem.set("ctrlrange", rotor_ctrlrange)
        rotor_elem.set("gear", "0 0 1 0 0 " + str(float(m_f_rate) * float(rotor_axis_dict[rotor])))
        rotor_elem.set("site", rotor)
        actuator_elem.append(rotor_elem)
    ## joints
    for joint in joint_list:
        joint_elem = ET.Element("position")
        joint_elem.set("name", joint)
        joint_elem.set("kp", "40")
        joint_elem.set("joint", joint)
        actuator_elem.append(joint_elem)
    mujoco_root.append(actuator_elem)

    # sensors
    sensor_elem = ET.Element("sensor")
    acc = ET.Element("accelerometer")
    acc.set("name", "acc")
    acc.set("site", "fc")
    gyro = ET.Element("gyro")
    gyro.set("name", "gyro")
    gyro.set("site", "fc")
    mag = ET.Element("magnetometer")
    mag.set("name", "mag")
    mag.set("site", "fc")
    sensor_elem.append(acc)
    sensor_elem.append(gyro)
    sensor_elem.append(mag)
    mujoco_root.append(sensor_elem)

    # Robot-specific, MuJoCo-only contact mechanisms. The option is absent for
    # existing configurations, so their generated models remain unchanged.
    if model_config is not None:
        add_compliant_feet(mujoco_root, model_config.get("compliant_feet"))
        add_compliant_feet(mujoco_root, model_config.get("compliant_grasp_pads"))

    # include world
    mujoco_ros_control = rospack.get_path("mujoco_ros_control")
    world_path = os.path.join(mujoco_ros_control, "config/world.xml")
    rel_path = os.path.relpath(world_path, get_directory(mujoco_path))
    include_elem = ET.Element("include")
    include_elem.set("file", rel_path)
    mujoco_root.append(include_elem)

    # output modified mujoco model
    xmlstr = minidom.parseString(ET.tostring(mujoco_root)).toprettyxml(indent="  ")
    with open(mujoco_path, "w") as f:
        f.write(xmlstr)

    # remove brank line in xml
    cmd = "sed -i '/^[[:space:]]*$/d' {}".format(mujoco_path)
    run_subprocess(cmd)

    # remove intermediate urdf file
    # os.remove(urdf_path)


def convert_dae2obj(meshdir):
    mujoco_ros_control = rospack.get_path("mujoco_ros_control")
    cmd = "python {} {}".format(os.path.join(mujoco_ros_control, "scripts/convert.py"), meshdir)
    print(cmd)
    run_subprocess(cmd)


def remove_generated_meshes(meshdir):
    for foldername, subfolders, filenames in os.walk(meshdir):
        for filename in filenames:
            if filename.endswith("_meta.json"):
                meta_path = os.path.join(foldername, filename)
                try:
                    with open(meta_path) as f:
                        meta = json.load(f)
                    for sub in meta["sub_meshes"]:
                        obj_path = os.path.join(foldername, sub["file"])
                        if os.path.isfile(obj_path):
                            os.remove(obj_path)
                        if sub.get("texture"):
                            tex_path = os.path.join(foldername, sub["texture"])
                            if os.path.isfile(tex_path):
                                os.remove(tex_path)
                    os.remove(meta_path)
                except Exception:
                    pass


config_path = ""
if(len(sys.argv) == 2):
    config_path = sys.argv[1]
else:
    print("Variable error! Please run following command.\nrosrun mujoco_ros_control mujoco_model_generator.py absolute_path_to_config_file")
    sys.exit()

with open(config_path) as file:
    obj = yaml.safe_load(file)
    for package in obj["package"]:
        print(package)
        if package not in obj:
            print("Error: package '{}' not found as a key in YAML. Available keys: {}".format(
                package, [k for k in obj.keys() if k != "package"]))
            sys.exit(1)
        pkg_path = rospack.get_path(package)
        meshdir = os.path.join(pkg_path, obj[package]["meshdir"])
        if os.path.isdir(os.path.join(pkg_path, "mujoco")):
            shutil.rmtree(os.path.join(pkg_path, "mujoco"))
        convert_dae2obj(meshdir)
        for (input_path, filename) in zip(obj[package]["input"], obj[package]["filename"]):
            input_xacro_path = os.path.join(pkg_path, input_path)
            workdir_path = os.path.join(pkg_path, "mujoco", filename)
            output_urdf_path = os.path.join(workdir_path, "robot.urdf")

            os.makedirs(workdir_path)

            run_xacro(input_xacro_path, output_urdf_path)

            process_urdf(package, output_urdf_path, workdir_path)

            mujoco_path = os.path.join(workdir_path, "robot.xml")
            generate_xml(output_urdf_path, mujoco_path)

            process_xml(output_urdf_path, mujoco_path, obj[package])

        remove_generated_meshes(meshdir)
