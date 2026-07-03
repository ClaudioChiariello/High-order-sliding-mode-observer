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

    def PlotAtEnd(self, output_data, observed_data, time_data):

        
        state = np.array(output_data)
        observed = np.array(observed_data)
        time = np.array(time_data)

        self.plot(
            time,
            state[:, obs.VX],
            observed[:, obs.VX],
            ylabel="vx [m/s]",
            filename="forward_vel.png"
        )

        self.plot(
            time,
            state[:, obs.VY],
            observed[:, obs.VY],
            ylabel="vy [m/s]",
            filename="lateral_vel.png"
        )

        self.plot(
            time,
            state[:, obs.WZ],
            observed[:, obs.WZ],
            ylabel="wz [rad/s]",
            filename="angular_vel_z.png"
        )

        # self.plot(
        #     time,
        #     state[:, obs.DOT_WX],
        #     observed[:, obs.DOT_WX],
        #     ylabel="phi_u [rad]",
        #     filename="phi_u.png"
        # )

        self.plot(
            time,
            state[:, obs.ROLL],
            observed[:, obs.ROLL],
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