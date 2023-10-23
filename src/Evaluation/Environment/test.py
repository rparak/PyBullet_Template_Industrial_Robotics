# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../..')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/PyBullet.Core
import Lib.PyBullet.Core

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str

def main():
    """
    Description:
        ...
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    PyBullet_Robot_Cls = Lib.PyBullet.Core.Robot_Cls('URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf', 100)
    
    while PyBullet_Robot_Cls.is_connected == True:
        PyBullet_Robot_Cls.Step()

    # ...
    PyBullet_Robot_Cls.Disconnect()
    
if __name__ == '__main__':
    main()