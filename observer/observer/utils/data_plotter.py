import matplotlib.pyplot as plt
import os
import numpy as np
from observer.utils.states import obs_state as s

class DataPlotter:

    def __init__(self, save_path="figures"):

        self.save_path = save_path

        if not os.path.exists(save_path):
            os.makedirs(save_path)

        self.data = {}

    def PlotAtEnd(self, state_data, observed_data, time_data):

        state = np.array(state_data)
        observed = np.array(observed_data)
        time = np.array(time_data)

        self.plot(
            time,
            state[:, s.VX],
            observed[:, s.VX],
            ylabel="vx [m/s]",
            filename="forward_vel.png"
        )

        self.plot(
            time,
            state[:, s.VY],
            observed[:, s.VY],
            ylabel="vy [m/s]",
            filename="lateral_vel.png"
        )

        self.plot(
            time,
            state[:, s.WZ],
            observed[:, s.WZ],
            ylabel="wz [rad/s]",
            filename="angular_vel_z.png"
        )

        self.plot(
            time,
            state[:, s.PHI_U],
            observed[:, s.PHI_U],
            ylabel="phi_u [rad]",
            filename="phi_u.png"
        )

        self.plot(
            time,
            state[:, s.ROLL],
            observed[:, s.ROLL],
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

        plt.plot(time, real, label="Real")
        plt.plot(time, observed, label="Observed")

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