 
import os
from ament_index_python.packages import get_package_share_directory 
import pinocchio
import numpy as np
from observer.utils.states import enum_obs_state as s
import ctypes

class robot:
    def __init__(self):
        
        urdf_path = os.path.join(
            get_package_share_directory('truck_description'),
            'urdf',
            'truck.urdf'
        )
        self.model = pinocchio.buildModelFromUrdf(
            urdf_path
        )

        self.data = self.model.createData()
        # Robot configuration
        dq = pinocchio.utils.zero(self.model.nv)  # joint velocities
        self.mass, self.Ix, self.Iz = self.get_mass_properties()
        self.L = 0
        self.delta = 0

        self.phi = np.float32(0.0)
        lib_path = os.path.join("/home/user/ros2_ws/src/observer/matlab/codegen/lib/vehicle_dynamics_numeric", 'libvehicle_dynamics.so')
        self.lib = ctypes.CDLL(lib_path)

        self.lib.vehicle_dynamics_numeric.argtypes = [
            ctypes.POINTER(ctypes.c_double), # state_obs (6x1)
            ctypes.c_double,                 # Fx (scalar)
            ctypes.c_double,                 # Mz (scalar)
            ctypes.POINTER(ctypes.c_double), # dot_x_num output (6x1)
            ctypes.POINTER(ctypes.c_double), # J_x_num output (6x6 -> 36 flat)
            ctypes.POINTER(ctypes.c_double), # h_num output (6x1)
            ctypes.POINTER(ctypes.c_double)  # J_h_num output (6x6 -> 36 flat)
        ]

    def get_mass_properties(self):
        """
        Compute total mass and inertia around CoM.
        """

        total_mass = 0.0

        for inertia in self.model.inertias[:]:
            total_mass += inertia.mass

        q0 = pinocchio.neutral(self.model)

        #Global Center of Mass
        pinocchio.centerOfMass(
            self.model,
            self.data,
            q0
        )
        #Center of mass in world frame
        com_world_frame = self.data.com[0]

        # pinocchio.ccrba(self.model,
        #     self.data,
        #     self.joint_pos,
        #     self.joint_vel
        # )

        # Ig = self.data.Ig

        # m  = Ig.mass
        # Ix = Ig.inertia[0,0]
        # Iz = Ig.inertia[2,2]

        #creates an empty rigid-body inertia object with: mass and inertia equal to zero
        composite = pinocchio.Inertia.Zero()

        for i in range(1, self.model.njoints):
            # oMi gives you the pose of datas in world frame
            oMi = self.data.oMi[i]
            inertia_i = self.model.inertias[i]
            I_world_i = oMi.act(inertia_i)
            # I_world_i is now the object whose fields mass, inertia and CoM are in the world frame
            com_world_i = I_world_i.lever

            shifted = I_world_i.se3Action(
                pinocchio.SE3(np.eye(3), com_world_i - com_world_frame)
            )
            """se3Action usa il teorema di Hyugeins stein per passare dal calcolo dell'inerzia
            del link rispetto al suo CoM a quella rispetto al CoM del truck"""

            """Initially it sujested this, where jointPlacement gives you the pose of the links
            with respect the parent frame. Since I do then placement.translation - com_world, the 
            operation is correct since I am adding quantities in different frames
            
            placement = self.model.jointPlacements[i]

            shifted = inertia_i.se3Action(
                pin.SE3(np.eye(3), placement.translation - com_world)
            )"""

            composite += shifted

        inertia_matrix = composite.inertia

        Ix = inertia_matrix[0, 0]
        Iz = inertia_matrix[2, 2]

        return total_mass, Ix, Iz


    def getFurtherParameterFromUrdf(self, L, delta):
         self.L = L
         self.delta = delta



    def calculate_dynamics(self, state, Fx, Mz, use_dist, dt):
            """Wrapper to safely feed numpy arrays straight into the raw C memory blocks."""
            # Ensure data arrays are contiguous float64 types for C compatibility
            in_obs = np.ascontiguousarray(state, dtype=np.float64)

            # Allocate empty output buffers for the C function to write into
            dot_x = np.zeros(6, dtype=np.float64)
            J_x   = np.zeros(36, dtype=np.float64)
            h     = np.zeros(6, dtype=np.float64)
            J_h   = np.zeros(36, dtype=np.float64)

            # Invoke the native C function execution loop
            self.lib.vehicle_dynamics_numeric(
                in_obs.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                ctypes.c_double(Fx),
                ctypes.c_double(Mz),
                dot_x.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                J_x.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                h.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                J_h.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
            )

            if(use_dist):
                 dot_x[:s.VY] += np.sin(2*np.pi*0.5*dt)
                 dot_x[-1] = 0.8*np.sin(2*np.pi*0.05*dt)
            
            # Reshape flat 1D output buffers back into proper 2D matrices
            return dot_x, J_x.reshape((6, 6), order='F'), h, J_h.reshape((6, 6), order='F') 

    def dynamics(self, state, Fx, Mz, add_disturb):
        """
        state = [x, y, roll, yaw, vx, vy, p, r]
        """
        x, y, phi, psi, vx, vy, p, r = state
        m = self.mass
        Ix = self.Ix
        Iz = self.Iz
        C_alpha = 300e2  # Note: 300 N/rad is incredibly low for a real vehicle (usually ~40000+)
        Cr = 200000.0
        Kr = 800000
        g = -9.81
        h = 1

        # Disturbance configuration
        dist = 0.0
        if add_disturb:
            dist = np.sin(4.0 * np.pi / 180.0 * (2.0 * np.pi / 5)**2)

        # Kinematics
        dx = vx * np.cos(psi) - vy * np.sin(psi) 
        dy = vx * np.sin(psi) + vy * np.cos(psi)
        # dx = vx
        # dy = vy

        dphi = p
        dpsi = r
        wx, wz = self.convertToAngularVel(dphi, dpsi)
        #dot_wx, dot_wz = self.euler_accel_to_body_accel(dphi, dpsi, dp, dr)

        # Prevent division by zero safely
        vx_lim = np.where(np.abs(vx) < 0.1, np.where(vx >= 0, 0.1, -0.1), vx)

        # Steering Geometry (L = [Lf1, Lf2, Lr1, Lr2])
        L = np.array([3.65, 1.75, 2.0, 3.39])
        
        # Calculate Slip Angles (alpha) and Forces (F) vectorially
        # Front tires (0, 1) have steering angles; Rear tires (2, 3) do not
        delta_1 = (np.sum(L)) / 2 * (wz / vx_lim)
        delta_2 = (2*L[1] + L[2] + L[3]) / (2*L[0] + L[2] + L[3]) * delta_1
        delta = np.array([delta_1, delta_2])

        # Calculate Slip Angles (alpha) and Forces (F) (Sostituito r -> wz)
        alpha = np.zeros(4, dtype='float32')
        alpha[0] = delta[0] - (vy + L[0] * wz) / vx_lim
        alpha[1] = delta[1] - (vy + L[1] * wz) / vx_lim
        alpha[2] = - (vy - L[2] * wz) / vx_lim
        alpha[3] = - (vy - L[3] * wz) / vx_lim

        # Without these forces the longitudinal velocity vx decrease
        F = C_alpha * alpha

        # Front forces projected onto the lateral axis
        F_lateral_total = F[0] * np.cos(delta[0]) + F[1] * np.cos(delta[1]) + F[2] + F[3]
        
        # Yaw moment from tire lateral forces
        # Moment = F_y * distance (Front adds positive yaw moment, rear resists it)
        tire_yaw_moment = (F[0] * np.cos(delta[0]) * L[0] + 
                        F[1] * np.cos(delta[1]) * L[1] - 
                        F[2] * L[2] - 
                        F[3] * L[3])

        dvx = Fx / m + vy * wz 
        dvy = -vx * wz + (F_lateral_total) / m
        dot_wx = (m * dvy * h + m * g * np.sin(phi) - Cr * wx - Kr * phi) / Ix
        dot_wz = (Mz + tire_yaw_moment) / Iz

        dot_x = np.array([dx, dy, dphi, dpsi, dvx, dvy, dot_wx, dot_wz])
        
        return dot_x


       #self.outputFunction(dot_x, dot_wx, dot_wz)
        
 



    def compute_accelerations_jacobian(self, state_obs, Fx, Mz, add_disturb, eps=1e-6):
            """
            Calcola lo Jacobiano (4x6) di [dvx, dvy, dot_wx, dot_wz] 
            rispetto allo stato dell'osservatore: [phi, wx, vy, wz, vx, phi_u]
            """
            # Ordine degli stati dell'osservatore: [phi, wx, vy, wz, vx, phi_u]
            Jacobian = np.zeros((4, 6))
            
            for i in range(6):
                # Crea due stati perturbati (+eps e -eps)
                state_plus = np.array(state_obs, dtype=float)
                state_minus = np.array(state_obs, dtype=float)
                
                state_plus[i] += eps
                state_minus[i] -= eps
                
                # Valuta le accelerazioni nel punto +eps
                acc_plus = self._get_only_accelerations(state_plus, Fx, Mz, add_disturb)
                # Valuta le accelerazioni nel punto -eps
                acc_minus = self._get_only_accelerations(state_minus, Fx, Mz, add_disturb)
                
                # Differenza finita centrale per la colonna i-esima
                Jacobian[:, i] = (acc_plus - acc_minus) / (2.0 * eps)
                
            return Jacobian


    def convertToAngularVel(self, dp, dr):
        T = np.array([
            [1.0, 0.0],
            [0.0, np.cos(self.phi)]
        ])
        
        euler_rates = np.array([dp, dr])
        body_rates = T @ euler_rates
        wx = body_rates[0]
        wz = body_rates[1]
        
        return wx, wz
    

    def euler_accel_to_body_accel(self, dot_roll, dot_yaw, ddot_roll, ddot_yaw):
        """
        Transforms Euler accelerations (ddot_roll, ddot_yaw) into 
        body angular accelerations (dot_omega_x, dot_omega_z).
        """
        # Matrix A (The standard transformation matrix)
        A = np.array([
            [1.0, 0.0],
            [0.0, np.cos(self.phi)]
        ])
        
        # Matrix A_dot (The time-derivative of the transformation matrix)
        A_dot = np.array([
            [0.0, 0.0],
            [0.0, -dot_roll * np.sin(self.phi)]
        ])
        
        euler_rates = np.array([dot_roll, dot_yaw])
        euler_accels = np.array([ddot_roll, ddot_yaw])
        
        # Acceleration formula: dot_omega = A * euler_accels + A_dot * euler_rates
        body_accels = (A @ euler_accels) + (A_dot @ euler_rates)
        
        dot_omega_x = body_accels[0]
        dot_omega_z = body_accels[1]
        
        return dot_omega_x, dot_omega_z