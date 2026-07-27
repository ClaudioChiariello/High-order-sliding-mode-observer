#!/usr/bin/env python3

import time
import numpy as np
import mujoco
import mujoco.viewer


# ---------------------------------------------------------------------
# Load model
# ---------------------------------------------------------------------

MODEL_PATH = "/home/user/ros2_ws/src/observer/worlds/mujoco_scene.xml"      # <-- change to your model

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)


# ---------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------

def print_joint_state():
    print("Joint positions:")
    print(data.qpos)

    print("Joint velocities:")
    print(data.qvel)

    print("Actuator forces:")
    print(data.qfrc_actuator)


def print_body_pose(body_id):

    pos = data.xpos[body_id]
    rot = data.xmat[body_id].reshape(3, 3)

    print(f"{body_id:2d} {"base_link"}")
    print(" position:", pos)
    print(" rotation:")
    print(rot)


def print_external_forces(body_id):

    force = data.xfrc_applied[body_id]

    if np.linalg.norm(force) > 1e-8:
        print("base_link", force)


def print_contacts():

    if data.ncon == 0:
        return

    print("\nContacts:", data.ncon)

    force = np.zeros(6)

    for i in range(data.ncon):

        contact = data.contact[i]

        # compute the force associated to the contact number i
        mujoco.mj_contactForce(
            model,
            data,
            i,
            force,
        ) 
        # Each contact stores two geometries. contact.geom1 gives you the geometry index, but we care about the body, not the geom. 
        # This function maps the geom id to the body id
        body1 = model.geom_bodyid[contact.geom1]
        body2 = model.geom_bodyid[contact.geom2]

        # converts body id into names
        name1 = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            body1,
        )

        name2 = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            body2,
        )

        print(f"{name1} <-> {name2}")
        print("force:", force[:3])
        print("torque:", force[3:])


# ---------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------

def main():
    
    with mujoco.viewer.launch_passive(model, data) as viewer:

        body_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "base_link"
        )
        count = 0
        while viewer.is_running():


            data.xfrc_applied[body_id] = np.array([
                    10000, 0, 0,   # force (N)
                    0, 0, 0      # torque (Nm)
            ])
            mujoco.mj_step(model, data)

            if count % 100 == 0:
                print_joint_state()
                print_body_pose(body_id)
                print_external_forces(body_id)
                print_contacts()

            viewer.sync()

            time.sleep(0.02)
            count +=1



if __name__ == "__main__":
    main()

