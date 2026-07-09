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

    def PlotAtEnd(self, sate_data, output_data, observed_state_data, observed_output_data, gazebo_state, gazebo_output, time_data, show_gazebo=True):
        
        state = np.array(sate_data)
        output = np.array(output_data)
        observed_state = np.array(observed_state_data)
        observed_out = np.array(observed_output_data)
        time = np.array(time_data)
        
        # Convert Gazebo datasets if they are provided
        gz_state = np.array(gazebo_state) if len(gazebo_state) > 0 is not None else None
        gz_out = np.array(gazebo_output) if len(gazebo_output) > 0 is not None else None

        # 1. Forward Velocity (VX)
        self.plot(
            time,
            output[:, enum_outputs.VX],
            observed_out[:, enum_outputs.VX],
            ylabel="vx [m/s]",
            filename="forward_vel.png",
            gazebo=gz_out[:, enum_outputs.VX] if (show_gazebo and gz_out is not None) else None
        )

        # 2. Lateral Acceleration (ACC_Y)
        self.plot(
            time,
            output[:, enum_outputs.ACC_Y],
            observed_out[:, enum_outputs.ACC_Y],
            ylabel="ay [m/s^2]",
            filename="LAteral_ACC.png",
            gazebo=gz_out[:, enum_outputs.ACC_Y] if (show_gazebo and gz_out is not None) else None
        )

        # 3. Angular Velocity Z (WZ)
        self.plot(
            time,
            output[:, enum_outputs.WZ],
            observed_out[:, enum_outputs.WZ],
            ylabel="wz [rad/s]",
            filename="angular_vel_z.png",
            gazebo=gz_out[:, enum_outputs.WZ] if (show_gazebo and gz_out is not None) else None
        )

        # 4. Angular Acceleration X (DOT_WX)
        self.plot(
            time,
            output[:, enum_outputs.DOT_WX],
            observed_out[:, enum_outputs.DOT_WX],
            ylabel="dot_WX [rad/s^2]",
            filename="dot_Wx.png",
            gazebo=gz_out[:, enum_outputs.DOT_WX] if (show_gazebo and gz_out is not None) else None
        )

        # 5. Lateral Velocity (VY)
        self.plot(
            time,
            state[:, enum_obs_state.VY],
            observed_state[:, enum_obs_state.VY], # Kept as observed_out to match your original configuration
            ylabel="vy [m/s]",
            filename="Lateral_vel.png",
            gazebo=gz_state[:, enum_obs_state.VY] if (show_gazebo and gz_state is not None) else None
        )

        # 6. Roll Orientation (PHI_TOT)
        self.plot(
            time,
            output[:, enum_outputs.ROLL],
            observed_out[:, enum_outputs.ROLL],
            ylabel="phi_tot [rad]",
            filename="phi_tot.png",
            gazebo=gz_out[:, enum_outputs.ROLL] if (show_gazebo and gz_out is not None) else None
        )

    def plot(self, time, real, observed, ylabel, filename, gazebo=None):
        plt.figure(figsize=(10, 6))

        # Core Plots
        plt.plot(time, real, label="Real", linewidth=2.5, color="royalblue")
        plt.plot(time, observed, label="Observed", linestyle="--", linewidth=1.5, color="darkorange")

        # Conditional Gazebo Plot
        if gazebo is not None:
            plt.plot(time, gazebo, label="Gazebo Ground Truth", linestyle="-.", linewidth=1.5, color="forestgreen")

        plt.xlabel("time [s]")
        plt.ylabel(ylabel)

        plt.grid(True, linestyle=":", alpha=0.6)
        plt.legend()

        save_file = os.path.join(self.save_path, filename)
        plt.savefig(save_file, dpi=300, bbox_inches="tight")
        plt.close()

        print(f"Saved: {save_file}")