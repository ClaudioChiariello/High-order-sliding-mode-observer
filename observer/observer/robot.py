 
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
    
    def get_mass_properties(self):
        """
        Compute total mass and inertia around CoM.
        """

        total_mass = 0.0

        for inertia in self.model.inertias[1:]:
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

    def dynamics(self, state, Fx, Mz, add_disturb):
            """
            state =
            [x,y,roll,yaw,vx,vy,p,r]
            """

            x, y, phi, psi, vx, vy, p, r = state
            epsilon = 0.0
            h = 1.0
            Cr = 200000.0
            Kr = 800000
            g = -9.81
            m = self.mass
            Ix = self.Ix
            Iz = self.Iz
            dist = 0.0

            if(add_disturb):
                 dist = np.sin(4.0*np.pi/180.0*(2.0*np.pi/5)**2)

            dx = vx*np.cos(psi) - vy*np.sin(psi) 
            dy = vx*np.sin(psi) + vy*np.cos(psi)
            
            dphi = p
            dpsi = r
           
            dvx = Fx/m + r*vy  + dist
            #dvy = Fy/m - r*vx
            dvy = -r*vx + dist

            dp = (m*dvy*h + m*g*np.sin(phi + epsilon) -Cr*dphi - Kr*phi )/Ix
            dr = (Mz)/Iz

            return np.array([
                dx,
                dy,
                dphi,
                dpsi,
                dvx,
                dvy,
                dp,
                dr
            ])

