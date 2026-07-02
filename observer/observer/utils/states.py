from enum import IntEnum

class state(IntEnum):
    X = 0
    Y = 1
    ROLL = 2
    YAW = 3
    VX = 4
    VY = 5
    WX = 6
    WZ = 7

class obs_state(IntEnum):
    ROLL = 0
    WX = 1
    VY = 2
    WZ = 3
    VX = 4
    PHI_U = 5