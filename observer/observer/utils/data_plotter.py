import matplotlib.pyplot as plt
import os
import numpy as np
from observer.utils.states import obs_state as obs
from observer.utils.states import outputs as out

class DataPlotter:

    def __init__(self, save_path="figures"):

        self.save_path = save_path

        if not os.path.exists(save_path):
            os.makedirs(save_path)

        self.data = {}

    def PlotAtEnd(self, sate_data, output_data, observed_state_data, observed_output_data, time_data):

        state = np.array(sate_data)
        output = np.array(output_data)
        observed_state = np.array(observed_state_data)
        observed_out = np.array(observed_output_data)
        time = np.array(time_data)

        self.plot(time,
            output[:, out.VX],
            observed_out[:, out.VX], ylabel="vx [m/s]", filename="forward_vel.png"
        )

        self.plot(
            time,
            output[:, out.ACC_Y],
            observed_out[:, out.ACC_Y],
            ylabel="ay [m/s^2]",
            filename="LAteral_ACC.png"
        )

        self.plot(
            time,
            state[:, obs.VY],
            observed_state[:, obs.VY],
            ylabel="vy [m/s]",
            filename="Lateral_vel.png"
        )


        self.plot(
            time,
            output[:, out.WZ],
            observed_out[:, out.WZ],
            ylabel="wz [rad/s]",
            filename="angular_vel_z.png"
        )

        self.plot(
            time,
            output[:, out.DOT_WX],
            observed_out[:, out.DOT_WX],
            ylabel="dot_WX [rad/s^2]",
            filename="dot_Wx.png"
        )

        self.plot(
            time,
            state[:, obs.WX],
            observed_state[:, obs.WX],
            ylabel="WX [rad/s]",
            filename="WX.png"
        )

        self.plot(
            time,
            output[:, out.ROLL],
            observed_out[:, out.ROLL],
            ylabel="phi_tot [rad]",
            filename="phi_tot.png"
        )


    def plot(self,
         time,
         real,
         observed,
         ylabel,
         filename):

        plt.figure(figsize=(10, 6))

        plt.plot(time, real, label="Real", linewidth=2.5)
        plt.plot(time, observed, label="Observed", linestyle="--", linewidth=1.5)

        plt.xlabel("time [s]")
        plt.ylabel(ylabel)

        plt.grid(True)
        plt.legend()

        save_file = os.path.join(
            self.save_path,
            filename
        )

        plt.savefig(
            save_file,
            dpi=300,
            bbox_inches="tight"
        )

        plt.close()

        print(f"Saved: {save_file}")