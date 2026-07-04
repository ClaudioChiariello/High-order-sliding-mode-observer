from enum import IntEnum

class obs_state(IntEnum):
    ROLL = 0
    WX = 1
    VY = 2
    WZ = 3
    VX = 4
    PHI_U = 5

class outputs(IntEnum):
    ROLL = 0
    ACC_Y = 1
    WX = 2
    DOT_WX = 3
    WZ = 4
    VX = 5