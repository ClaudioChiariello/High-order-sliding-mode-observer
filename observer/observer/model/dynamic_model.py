 
import os
import numpy as np
import ctypes
from observer.utils.states import enum_obs_state, enum_outputs

class DynamicModel:


    def __init__(self):

        lib_path_real = os.path.join("/home/user/ros2_ws/src/observer/matlab/codegen/lib/vehicle_dynamics_real", 'libvehicle_dynamics_real.so')

        lib_path_obs = os.path.join("/home/user/ros2_ws/src/observer/matlab/codegen/lib/vehicle_dynamics_obs", 'libvehicle_dynamics_obs.so')
        
        self.lib_real_model = ctypes.CDLL(lib_path_real)
        self.lib_obs_model = ctypes.CDLL(lib_path_obs)

        # PID Terms
        self.e_int = np.zeros(2,dtype="float32")
        self.previous_e = np.zeros(2,dtype="float32")

        self.lib_real_model.vehicle_dynamics_real.argtypes = [
            ctypes.POINTER(ctypes.c_double), # real_state (6x1)
            ctypes.c_double,                 # Fx (scalar)
            ctypes.c_double,                 # Mz (scalar)
            ctypes.POINTER(ctypes.c_double), # dot_x_real output (6x1)
            ctypes.POINTER(ctypes.c_double), # J_x_num output (6x6 -> 36 flat)
            ctypes.POINTER(ctypes.c_double), # h_num output (6x1)
            ctypes.POINTER(ctypes.c_double)  # J_h_num output (6x6 -> 36 flat)
        ]

        self.lib_obs_model.vehicle_dynamics_obs.argtypes = [
            ctypes.POINTER(ctypes.c_double), # state_obs (6x1)
            ctypes.c_double,                 # Fx (scalar)
            ctypes.c_double,                 # Mz (scalar)
            ctypes.POINTER(ctypes.c_double), # dot_x_obs output (6x1)
            ctypes.POINTER(ctypes.c_double), # J_x_num output (6x6 -> 36 flat)
            ctypes.POINTER(ctypes.c_double), # h_num output (6x1)
            ctypes.POINTER(ctypes.c_double)  # J_h_num output (6x6 -> 36 flat)
        ]


    def calculate_real_dynamics(self, state, Fx, Mz, use_dist, dt):
            """Wrapper to safely feed numpy arrays straight into the raw C memory blocks."""
            # Ensure data arrays are contiguous float64 types for C compatibility
            in_obs = np.ascontiguousarray(state, dtype=np.float64)

            # Allocate empty output buffers for the C function to write into
            dot_x = np.zeros(6, dtype=np.float64)
            J_x   = np.zeros(36, dtype=np.float64)
            h     = np.zeros(6, dtype=np.float64)
            J_h   = np.zeros(36, dtype=np.float64)

            # Invoke the native C function execution loop

            self.lib_real_model.vehicle_dynamics_real(
                in_obs.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                ctypes.c_double(Fx),
                ctypes.c_double(Mz),
                dot_x.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                J_x.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                h.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                J_h.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
            )
            
            if(use_dist):
                phi_u = np.deg2rad(45) * np.sin(2*np.pi*(1/5)*dt)
                dphi_u = np.deg2rad(45) * 1/(2*np.pi*(1/5)) * (-np.cos(2*np.pi*(1/5)*dt))
                dot_x += np.sin(2*np.pi*1.25*dt)
                dot_x[-1] = dphi_u
            
            # Reshape flat 1D output buffers back into proper 2D matrices
            return dot_x, J_x.reshape((6, 6), order='F'), h, J_h.reshape((6, 6), order='F') 


    def calculate_obs_dynamics(self, state, Fx, Mz):
        
        """Wrapper to safely feed numpy arrays straight into the raw C memory blocks."""
        # Ensure data arrays are contiguous float64 types for C compatibility
        in_obs = np.ascontiguousarray(state, dtype=np.float64)

        # Allocate empty output buffers for the C function to write into
        dot_x = np.zeros(6, dtype=np.float64)
        J_x   = np.zeros(36, dtype=np.float64)
        h     = np.zeros(6, dtype=np.float64)
        J_h   = np.zeros(36, dtype=np.float64)

        # Invoke the native C function execution loop

        self.lib_obs_model.vehicle_dynamics_obs(
            in_obs.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            ctypes.c_double(Fx),
            ctypes.c_double(Mz),
            dot_x.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            J_x.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            h.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            J_h.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        )
        
        # Reshape flat 1D output buffers back into proper 2D matrices
        return dot_x, J_x.reshape((6, 6), order='F'), h, J_h.reshape((6, 6), order='F') 


    def joint_states_callback(self, joint_msg):

        a = joint_msg

    def PdController(self, current_state, des_vel_x, des_omega_z, dt):

        Kp = np.array([100000, 2000000])
        Ki = np.array([100, 100.0])
        Kd = np.array([100, 10])

        vx_des, _, des_omega_z =  np.array([des_vel_x, 0, des_omega_z])
        
        control_error = np.array([
            vx_des - current_state[enum_obs_state.VX],
            des_omega_z - current_state[enum_obs_state.WZ]
        ])

        derivative_term = Kd*((control_error - self.previous_e)/dt)
        integral_term = Ki*self.e_int

        u = Kp*control_error + integral_term  
    
        self.previous_e = control_error
        self.e_int += control_error*dt

        return u