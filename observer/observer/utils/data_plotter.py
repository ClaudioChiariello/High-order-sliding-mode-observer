import matplotlib.pyplot as plt
import os
import numpy as np
from observer.utils.states import enum_obs_state, enum_outputs

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
            output[:, enum_outputs.VX],
            observed_out[:, enum_outputs.VX], ylabel="vx [m/s]", filename="forward_vel.png"
        )

        self.plot(
            time,
            output[:, enum_outputs.ACC_Y],
            observed_out[:, enum_outputs.ACC_Y],
            ylabel="ay [m/s^2]",
            filename="LAteral_ACC.png"
        )

        self.plot(
            time,
            state[:, enum_obs_state.VY],
            observed_state[:, enum_obs_state.VY],
            ylabel="vy [m/s]",
            filename="Lateral_vel.png"
        )


        self.plot(
            time,
            output[:, enum_outputs.WZ],
            observed_out[:, enum_outputs.WZ],
            ylabel="wz [rad/s]",
            filename="angular_vel_z.png"
        )

        self.plot(
            time,
            output[:, enum_outputs.DOT_WX],
            observed_out[:, enum_outputs.DOT_WX],
            ylabel="dot_WX [rad/s^2]",
            filename="dot_Wx.png"
        )

        self.plot(
            time,
            state[:, enum_obs_state.WX],
            observed_state[:, enum_obs_state.WX],
            ylabel="WX [rad/s]",
            filename="WX.png"
        )

        self.plot(
            time,
            output[:, enum_outputs.ROLL],
            observed_out[:, enum_outputs.ROLL],
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