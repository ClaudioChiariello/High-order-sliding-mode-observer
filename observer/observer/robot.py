 
import os
from ament_index_python.packages import get_package_share_directory 
import pinocchio
import numpy as np

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
        # dx = vx * np.cos(psi) - vy * np.sin(psi) 
        # dy = vx * np.sin(psi) + vy * np.cos(psi)
        dx = vx
        dy = vy

        dphi = p
        dpsi = r

        # Prevent division by zero safely
        vx_lim = np.where(np.abs(vx) < 0.1, np.where(vx >= 0, 0.1, -0.1), vx)

        # Steering Geometry (L = [Lf1, Lf2, Lr1, Lr2])
        L = np.array([3.65, 1.75, 2.0, 3.39])
        
        delta_1 = (np.sum(L)) / 2 * (r / vx_lim)
        delta_2 = (2*L[1] + L[2] + L[3]) / (2*L[0] + L[2] + L[3]) * delta_1
        delta = np.array([delta_1, delta_2])

        # Calculate Slip Angles (alpha) and Forces (F) vectorially
        # Front tires (0, 1) have steering angles; Rear tires (2, 3) do not
        alpha = np.zeros(4, dtype='float32')
        alpha[0] = delta[0] - (vy + L[0] * r) / vx_lim
        alpha[1] = delta[1] - (vy + L[1] * r) / vx_lim
        alpha[2] = - (vy - L[2] * r) / vx_lim
        alpha[3] = - (vy - L[3] * r) / vx_lim

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

        # Accelerations
        dvx = Fx / m + vy * r 
        dvy = -vx * r + (F_lateral_total) / m
        dp = (m * dvy * h + m * g *np.sin(phi) -Cr * dphi - Kr * phi ) / Ix
        dr = (Mz + tire_yaw_moment) / Iz  # Restoring tire moment naturally opposes Mz if signs match physics

        return np.array([dx, dy, dphi, dpsi, dvx, dvy, dp, dr])