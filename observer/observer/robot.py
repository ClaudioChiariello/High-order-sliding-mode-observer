 
import os
from ament_index_python.packages import get_package_share_directory 
import pinocchio

 
urdf_path = os.path.join(
    get_package_share_directory('truck_description'),
    'urdf',
    'truck.urdf'
)
model = pinocchio.buildModelFromUrdf(
    urdf_path
)

data = model.createData()
# Robot configuration
q = pinocchio.neutral(model)      # joint positions
dq = pinocchio.utils.zero(model.nv)  # joint velocities


Mass = pinocchio.crba(
    model,
    data,
    q
)

Coriolis = pinocchio.computeCoriolisMatrix(
    model,
    data,
    q,
    dq
)

