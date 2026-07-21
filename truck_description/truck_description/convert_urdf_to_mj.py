import os
import xml.etree.ElementTree as ET
import mujoco
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def add_actuator_tag(mjcf_root):
    
    actuator_elem = ET.SubElement(mjcf_root, "actuator")
    # Steering actuators (Position control)
    steer_joints = [
        "left_steer_joint_a1", "right_steer_joint_a1",
        "left_steer_joint_a2", "right_steer_joint_a2"
    ]

    for j in steer_joints:
        ET.SubElement(actuator_elem, "position", {
            "name": j, "joint": j, "kp": "1000", "kv": "1000"
        })


    for joint in mjcf_root.iter("joint"):
        if joint.get("name") in steer_joints:
            joint.set("actuatorfrcrange", "-100000 100000")

    # Wheel drive actuators (Velocity control)
    rot_joints = [
        "left_rot_joint_a1", "right_rot_joint_a1",
        "left_rot_joint_a2", "right_rot_joint_a2",
        "left_rot_joint_a3", "right_rot_joint_a3",
        "left_rot_joint_a4", "right_rot_joint_a4"
    ]
    for j in rot_joints:
        ET.SubElement(actuator_elem, "velocity", {
            "name": j, "joint": j, "kv": "1200"
        })
    ET.indent(actuator_elem, space="  ")



def add_sensor_tag(mjcf_root):
    sensor_elem = ET.SubElement(mjcf_root, "sensor")
    ET.SubElement(sensor_elem, "framepos", {
        "name": "body_pos", "objtype": "body", "objname": "left_wheel_axis3_1"
    })

    ET.SubElement(sensor_elem, "framequat", {
        "name": "body_quat", "objtype": "body", "objname": "left_wheel_axis3_1"
    })

    ET.indent(sensor_elem, space="  ")
 
    ET.SubElement(sensor_elem, "frameangvel", {
        "name": "body_angvel",
        "objtype": "body",
        "objname": "base_link"   
    })


def remove_contact_meshed(mjcf_root):
    """Separates collision geoms into group 3 (hidden) and visual geoms into group 1."""

    for geom in mjcf_root.iter("geom"):
        geom_name = geom.get("name", "").lower()
        contype = geom.get("contype")
        conaffinity = geom.get("conaffinity")
        density = geom.get("density")
        geom_type = geom.get("type", "")

        # Check if explicitly defined as a collision shape
        is_collision = (
            "collision" in geom_name 
            or (contype is not None and contype != "0")
            or (conaffinity is not None and conaffinity != "0")
            or geom_type in ["box", "sphere", "cylinder", "capsule", "ellipsoid"]
        )

        # Visual meshes in URDF-to-MJCF conversions typically have density="0"
        is_visual = (
            contype == "0" 
            or conaffinity == "0" 
            or density == "0"
        ) and "collision" not in geom_name

        if is_collision and not is_visual:
            geom.set("group", "3")  # Hidden collision group
        else:
            geom.set("group", "1")  # Visible layer


def main():

    truck_description = get_package_share_directory('truck_description')
    
    # 1. Load URDF file
    urdf_path = os.path.join(truck_description, "urdf", "truck.urdf")
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # 2. Inject compiler rules into URDF tree
    mujoco_elem = root.find("mujoco")
    if mujoco_elem is None:
        mujoco_elem = ET.SubElement(root, "mujoco")
    
    ET.SubElement(mujoco_elem, "compiler", {
        "fusestatic": "false",
        "discardvisual": "false"
    })

    # Clean file:// paths
    modified_urdf_str = ET.tostring(root, encoding="unicode").replace("file://", "")

    # 3. Parse with MuJoCo and save base MJCF XML
    model = mujoco.MjModel.from_xml_string(modified_urdf_str)
    observer_share = os.path.join(get_package_share_directory('observer'), "worlds")
    output_path = os.path.join(observer_share, "truck_mujoco.xml")
    # The file is converted into a mujoco file, and can be modified later adding actuators, but not yet saved
    mujoco.mj_saveLastXML(output_path, model)

    # 4. Post-process: Inject <actuator> and <sensor> tags into generated XML
    mjcf_tree = ET.parse(output_path)
    mjcf_root = mjcf_tree.getroot()

    worldbody = mjcf_root.find("worldbody")
    # Add the freejoint to allow the robot to move
    if worldbody is not None:
        # Get the main robot body (first body directly inside worldbody)
        root_body = worldbody.find("body")
        if root_body is not None:
            # Check if it doesn't already have a freejoint
            if root_body.find("freejoint") is None:
                # Create <freejoint name="root_freejoint"/> inside the base body
                ET.SubElement(root_body, "freejoint", {"name": "root_freejoint"})


    add_actuator_tag(mjcf_root)
    add_sensor_tag(mjcf_root)
    remove_contact_meshed(mjcf_root)

    # 5. Save updated XML
    mjcf_tree.write(output_path, encoding="utf-8", xml_declaration=True)

    ws_root = os.path.abspath(os.path.join(get_package_prefix('observer'), '..', '..'))
    observer_src = os.path.join(ws_root, 'src', 'observer', "worlds", "truck_mujoco.xml")
    mjcf_tree.write(observer_src, encoding="utf-8", xml_declaration=True)
    
    print(f"Successfully generated MuJoCo model with actuators & sensors at: {output_path}")




if __name__ == '__main__':
    main()