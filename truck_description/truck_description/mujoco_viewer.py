import time

import mujoco
import mujoco.viewer


def main():

    model_path = "/home/user/ros2_ws/src/observer/worlds/truck.xml"

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:

        # Show body coordinate frames
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY

        # Visualization options
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CAMERA] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_LIGHT] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_INERTIA] = True

        while viewer.is_running():

            mujoco.mj_step(model, data)
            viewer.sync()

            time.sleep(model.opt.timestep)
        for i in range(model.nbody):
            print(i, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i))

if __name__ == "__main__":
    main()