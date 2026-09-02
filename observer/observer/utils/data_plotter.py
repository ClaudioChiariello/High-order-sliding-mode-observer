import matplotlib.pyplot as plt
import os
import numpy as np
from observer.utils.states import enum_obs_state, enum_outputs
import pandas as pd

class DataPlotter:

    def __init__(self, save_path="figures"):
        self.save_path = save_path

        if not os.path.exists(save_path):
            os.makedirs(save_path)

        self.data = {}


    def save_to_csv(
            self,
            time_data,
            state_data,
            output_data,
            observed_state_data,
            observed_output_data,
            phi_s,
            gazebo_state=None,
            gazebo_output=None,
            filename="simulation_data.csv",
        ):
            """Exports all time-series signals to a structured CSV for MATLAB processing."""
            data_dict = {"time": np.array(time_data).squeeze()}

            # 1. State Vectors (Real vs. Observed)
            state = np.array(state_data)
            observed_state = np.array(observed_state_data)
            for state_enum in enum_obs_state:
                data_dict[f"state_real_{state_enum.name}"] = state[:, state_enum.value]
                data_dict[f"state_obs_{state_enum.name}"] = observed_state[:, state_enum.value]

            # 2. Output Vectors (Real vs. Observed)
            output = np.array(output_data)
            observed_output = np.array(observed_output_data)
            for out_enum in enum_outputs:
                data_dict[f"output_real_{out_enum.name}"] = output[:, out_enum.value]
                data_dict[f"output_obs_{out_enum.name}"] = observed_output[:, out_enum.value]

            # 3. Additional Signals
            data_dict["phi_s"] = np.array(phi_s).squeeze()

            # 4. Optional Gazebo Data
            if gazebo_state is not None and len(gazebo_state) > 0:
                gz_state = np.array(gazebo_state)
                for state_enum in enum_obs_state:
                    data_dict[f"gz_state_{state_enum.name}"] = gz_state[:, state_enum.value]

            if gazebo_output is not None and len(gazebo_output) > 0:
                gz_out = np.array(gazebo_output)
                for out_enum in enum_outputs:
                    data_dict[f"gz_output_{out_enum.name}"] = gz_out[:, out_enum.value]

            # Convert to Pandas DataFrame and export to CSV
            df = pd.DataFrame(data_dict)
            filepath = os.path.join(self.save_path, filename)
            another_path = "/home/user/ros2_ws/src/observer/matlab/simulation_data.csv"
            df.to_csv(filepath, index=False)
            df.to_csv(another_path, index=False)
            print(f"Data saved to CSV: {filepath}")

    def PlotAtEnd(self, sate_data, output_data, observed_state_data, observed_output_data, gazebo_state, gazebo_output, time_data, phi_s, show_gazebo=True):
        
        state = np.array(sate_data)
        output = np.array(output_data)
        observed_state = np.array(observed_state_data)
        observed_out = np.array(observed_output_data)
        time = np.array(time_data)
        
        # Convert Gazebo datasets if they are provided
        gz_state = np.array(gazebo_state) if len(gazebo_state) > 0 else None
        gz_out = np.array(gazebo_output) if len(gazebo_output) > 0 else None

        # 1. Forward Velocity (VX)
        self.plot(
            time,
            output[:, enum_outputs.VX],
            observed_out[:, enum_outputs.VX],
            Title = "Forward velocity",
            ylabel="vx [m/s]",
            filename="forward_vel.png",
            gazebo=gz_out[:, enum_outputs.VX] if (show_gazebo and gz_out is not None) else None
        )

        # 2. Lateral Acceleration (ACC_Y)
        self.plot(
            time,
            output[:, enum_outputs.ACC_Y],
            observed_out[:, enum_outputs.ACC_Y],
            Title = "Lateral acceleration",
            ylabel="ay [m/s^2]",
            filename="LAteral_ACC.png",
            gazebo=gz_out[:, enum_outputs.ACC_Y] if (show_gazebo and gz_out is not None) else None
        )

        # 3. Angular Velocity Z (WZ)
        self.plot(
            time,
            output[:, enum_outputs.WZ],
            observed_out[:, enum_outputs.WZ],
            Title = "Angular velocity about z; wz",
            ylabel="wz [rad/s]",
            filename="angular_vel_z.png",
            gazebo=gz_out[:, enum_outputs.WZ] if (show_gazebo and gz_out is not None) else None
        )

        # 4. Angular Acceleration X (DOT_WX)
        self.plot(
            time,
            output[:, enum_outputs.DOT_WX],
            observed_out[:, enum_outputs.DOT_WX],
            Title = "Angular acceleration about x; dot_wx",
            ylabel="dot_WX [rad/s^2]",
            filename="dot_Wx.png",
            gazebo=gz_out[:, enum_outputs.DOT_WX] if (show_gazebo and gz_out is not None) else None
        )

        # 5. Lateral Velocity (VY)
        self.plot(
            time,
            state[:, enum_obs_state.VY],
            observed_state[:, enum_obs_state.VY], # Kept as observed_out to match your original configuration
            Title = "Lateral velocity Vy",
            ylabel="vy [m/s]",
            filename="Lateral_vel.png",
            gazebo=gz_state[:, enum_obs_state.VY] if (show_gazebo and gz_state is not None) else None
        )

        # 6. Roll Orientation (PHI_TOT)
        self.plot(
            time,
            output[:, enum_outputs.ROLL],
            observed_out[:, enum_outputs.ROLL],
            Title = "phi total",
            ylabel="phi_tot [rad]",
            filename="phi_tot.png",
            gazebo=gz_out[:, enum_outputs.ROLL] if (show_gazebo and gz_out is not None) else None
        )

        self.plot(
            time,
            state[:, enum_obs_state.PHI_U],
            observed_state[:, enum_obs_state.PHI_U],
            Title = "phi unsprung mass",
            ylabel="phi_u [rad]",
            filename="phi_u.png",
            gazebo=gz_out[:, enum_obs_state.PHI_U] if (show_gazebo and gz_out is not None) else None
        )

        self.plot(
            time,
            output[:, enum_outputs.WX],
            observed_out[:, enum_outputs.WX],
            Title = "Angular velocity about x",
            ylabel="wx [rad/s]",
            filename="omega_x.png",
            gazebo=gz_out[:, enum_outputs.ROLL] if (show_gazebo and gz_out is not None) else None
        )

        self.plot_just_one(
            time,
            phi_s,
            var_name = "phi_s",
            ylabel="phi_s [rad/s]",
            filename="phi_s.png",
        )

    def plot(self, time, real, observed, Title, ylabel, filename, gazebo=None):
        plt.figure(figsize=(10, 6))

        # Core Plots
        plt.plot(time, real, label="Real", linewidth=2.5, color="royalblue")
        plt.plot(time, observed, label="Observed", linestyle="--", linewidth=1.5, color="darkorange")

        # Conditional Gazebo Plot
        if gazebo is not None:
            plt.plot(time, gazebo, label="Gazebo Ground Truth", linestyle="-.", linewidth=1.5, color="forestgreen")

        plt.title(Title)
        plt.xlabel("time [s]")
        plt.ylabel(ylabel)

        plt.grid(True, linestyle=":", alpha=0.6)
        plt.legend()

        save_file = os.path.join(self.save_path, filename)
        plt.savefig(save_file, dpi=300, bbox_inches="tight")
        plt.close()

        print(f"Saved: {save_file}")



    def plot_just_one(self, time, x, var_name, ylabel, filename):
        plt.figure(figsize=(10, 6))

        # Core Plots
        plt.plot(time, x, label=var_name, linewidth=2.5, color="royalblue")
    
        plt.xlabel("time [s]")
        plt.ylabel(ylabel)
        plt.title(var_name)
        plt.grid(True, linestyle=":", alpha=0.6)
        plt.legend()
        save_file = os.path.join(self.save_path, filename)
        plt.savefig(save_file, dpi=300, bbox_inches="tight")
        plt.close()