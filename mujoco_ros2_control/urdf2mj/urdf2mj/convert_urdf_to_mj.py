import os
import xml.etree.ElementTree as ET
import mujoco
from ament_index_python.packages import get_package_share_directory, get_package_prefix

"""This script generate the scene.xml file that will be included from the ros2_control tag into the urdf at the path
truck_control/urdf/truc8x8.ros2_control"""

def convert_wheels_to_cylinders(mjcf_root, wheel_half_width="0.18"):
    """
    Converts wheel collision geoms from sphere shapes to cylinders.
    
    MuJoCo cylinders default to alignment along their local Z-axis.
    Since wheel rotation joints are along the Y-axis (axis="0 -1 0"),
    we rotate the cylinder by 90 degrees around the X-axis using 
    quat="0.707107 0.707107 0 0" so it aligns with the rotation axis.
    """
    wheel_keywords = ["wheel", "rot_joint", "axis1", "axis2", "axis3", "axis4"]

    for geom in mjcf_root.iter("geom"):
        geom_name = geom.get("name", "").lower()
        
        # Target wheel collision geoms
        if any(keyword in geom_name for keyword in wheel_keywords) and "collision" in geom_name:
            # Change geometry type to cylinder
            # geom.set("type", "cylinder")
            
            # # Extract current radius from size (e.g., "0.5645")
            # current_size = geom.get("size", "0.5645").split()
            # radius = current_size[0]
            
            # # Cylinder size requires: "radius half_width"
            # geom.set("size", f"{radius} {wheel_half_width}")
            
            # # Rotate cylinder's local Z axis to align with joint's Y axis
            # geom.set("quat", "0.707107 0.707107 0 0")
            
            # geom.set("quat", "0.707107 0.707107 0 0")

            geom.set("solimp", "0.9 0.95 0.001 0.5 2")  # default 
            # Il primo termine è il tempo di risposta del sistema a deformazioni. Quanto più è grande, tanto piu' il sistema impiegherà per rispondere a deformazioni. 
            # Tanto maggiore e', tanto più sara' soffice il corpo a contatto.
            # Se quindi lo alzo, durante la curva le ruote esterne penetrano di più e le ruote posteriori interne non saranno piu' a contatto
            geom.set("solref", "0.25 1") # Default "solref", "0.02 1"
            geom.set("margin", "0.05")  #the distance between the geom must be less than this margin to detect contact
            geom.set("gap", "0.00005")
            # condim="4"  computes tangential frictional + normal-force + torsional-friction
            geom.set("condim", "3") #just tangential friction forces and normal, I don't need torsional friction


def set_mujoco_solver(mjcf_root, solver_type="Newton"):
    """
    Sets the MuJoCo solver algorithm.
    Options: 'Newton', 'CG', or 'PGS'
    """
    option_elem = mjcf_root.find("option")
    
    # Create <option> if it doesn't exist yet
    if option_elem is None:
        option_elem = ET.SubElement(mjcf_root, "option")
        
    # Set the solver attribute
    option_elem.set("solver", solver_type)
    
    # Optional: For heavy contact models like multi-wheel trucks, 
    # boosting solver iterations prevents energy loss during turns:
    option_elem.set("iterations", "50")
    option_elem.set("tolerance", "1e-10")


def add_actuator_tag(mjcf_root):
    actuator_elem = ET.SubElement(mjcf_root, "actuator")
    
    # Steering actuators (Position control)
    steer_joints = [
        "left_steer_joint_a1", "right_steer_joint_a1",
        "left_steer_joint_a2", "right_steer_joint_a2"
    ]

    for j in steer_joints:
        ET.SubElement(actuator_elem, "position", {
            "name": j, "joint": j, "kp": "5e4", "kv": "1e4"
        })

    # Set force limit on steer joints
    for joint in mjcf_root.iter("joint"):
        if joint.get("name") in steer_joints:
            joint.set("actuatorfrcrange", "-1e8 1e8")
            joint.set("damping", "1e4")  # Prevents runaway lateral energy transfer
            joint.set("armature", "1e2") # Adds realistic rotor inertia       

    # Wheel drive actuators (Velocity control)
    rot_joints = [
        "left_rot_joint_a1", "right_rot_joint_a1",
        "left_rot_joint_a2", "right_rot_joint_a2",
        "left_rot_joint_a3", "right_rot_joint_a3",
        "left_rot_joint_a4", "right_rot_joint_a4"
    ]
    
    for j in rot_joints:
        ET.SubElement(actuator_elem, "velocity", {
            "name": j, 
            "joint": j, 
            "kv": "7000",
            "forcerange": "-2e7 2e7"  # <--- Fix: Force limit for velocity actuators
        })

    ET.indent(actuator_elem, space="  ")



"""Senza questa funzione, mi ritrovo che le steering wheel sono in contatto anche con il base_link. In questo modo, si creano delle forze di contatto che sembrano
    - O generare forze laterali che spingono il truck
    - Oppure generano attriti tra i link delle ruote e quando si deve girare, questi attriti bloccano in qualche modo le ruote di steering, facendo rallentare il truck
    Il problema l'ho risolto mettendo il base_link con una conaffinity che gli ipedisca di collidere con le steering wheel. Quella collisione bloccava il truck in qualche modo
    
    Aumentare la friction al link delle wheel anche aiutava, ma doveevo mettere numeri enormi. All'inizio pensavo che fosse che il tracking di velocità del controllore ros2 diminuiva
    migliorasse perché aveva cambiato l'attrito con il terreno, e più attrito aiutava il controllore a tenere il truck sul percorso, ma in realtà stavo aumentando l'attrito tra le ruote
    e il base link"""
def update_wheel_friction(mjcf_root):
    """Sets customized friction for wheel collision geoms to eliminate sideways scrubbing."""
    wheel_keywords = ["wheel", "rot_joint", "axis1", "axis2", "axis3", "axis4"]

    for geom in mjcf_root.iter("geom"):
        geom_name = geom.get("name", "").lower()
        
        # Target wheel collision geoms specifically
        if any(keyword in geom_name for keyword in wheel_keywords):
           
            geom.set("contype","2")  #Setto un contype e conaffinity del link delle ruote in modo che possa essere compatibile solo con quello del terreno, ma non con quello del base_link
            geom.set("conaffinity", "1")
 
            # Values: [sliding, torsional, rolling]
            # 1.1   -> High sliding grip so tires don't slide laterally
            # 0.005 -> Very low torsional friction so wheels twist/turn smoothly without dragging
            # 0.0001 -> Low rolling resistance for fast forward motion
            geom.set("friction", "0.9 0.01 0.002")

    # Setto il contype and conaffinity in modo che siano incompatibili da quelli delle ruote e del terreno, per dire che non devono collidere tra loro
    for body in mjcf_root.iter("body"):
        if body.get("name") == "base_link":
                    # Loop through all geoms that belong directly to base_link
                    for geom in body.findall("geom"):
 
                        # Optional: Only target collision geoms if group="3" or named "collision"
                        # If you want to update ALL geoms under base_link, remove this if-statement:
                        geom_name = geom.get("name", "").lower()
                        contype = geom.get("contype", "")
                        geom.set("group", "3")
                        # Exclude pure visual geoms (contype="0") unless intended
                        if contype != "0":
                            geom.set("contype", "4")      # Bitwise mask for base_link
                            geom.set("conaffinity", "4")  #
                        


def add_sensor_tag(mjcf_root):
    sensor_elem = ET.SubElement(mjcf_root, "sensor")
    ET.SubElement(sensor_elem, "framepos", {
        "name": "body_pos", "objtype": "body", "objname": "base_link"
    })

    ET.SubElement(sensor_elem, "framequat", {
        "name": "body_quat", "objtype": "body", "objname": "base_link"
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
            if geom_type == "cylinder":
                geom.set("group", "4")  # Set wheel cylinders to group 4
            else:
                geom.set("group", "3")
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

    set_mujoco_solver(mjcf_root)
    add_actuator_tag(mjcf_root)
    add_sensor_tag(mjcf_root)
    convert_wheels_to_cylinders(mjcf_root, wheel_half_width="0.18")  # <--- Converts sphere -> cylinder
    update_wheel_friction(mjcf_root) 
    remove_contact_meshed(mjcf_root)

    # 5. Save updated XML
    mjcf_tree.write(output_path, encoding="utf-8", xml_declaration=True)

    ws_root = os.path.abspath(os.path.join(get_package_prefix('observer'), '..', '..'))
    observer_src = os.path.join(ws_root, 'src', 'observer', "worlds", "truck_mujoco.xml")
    mjcf_tree.write(observer_src, encoding="utf-8", xml_declaration=True)
    
    print(f"Successfully generated MuJoCo model with actuators & sensors at: {output_path}")


if __name__ == '__main__':
    main()