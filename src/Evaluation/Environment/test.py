# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../..')
# OS (Operating system interfaces)
import os
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
CONST_ROBOT_TYPE = Parameters.ABB_IRB_120_Str

def main():
    """
    Description:
        ...
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('PyBullet_Template')[0] + 'PyBullet_Template'

    # ..
    PyBullet_Robot_Cls = Lib.PyBullet.Core.Robot_Cls(Robot_Str, f'{project_folder}/URDFs/Robots/ABB_IRB_120/ABB_IRB_120.urdf', 
                                                     {'Enable_GUI': 0, 'fps': 100, 'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance':1.3, 
                                                                                              'Position': [0.05, -0.10, 0.06]}})
    
    # ...
    PyBullet_Robot_Cls.Reset('Zero')

    print(PyBullet_Robot_Cls.T_EE)

    # ...
    while PyBullet_Robot_Cls.is_connected == True:
        #PyBullet_Robot_Cls.Set_Absolute_Joint_Position(Robot_Str.Theta.Zero, 0.0, 5.0)
        #PyBullet_Robot_Cls.Step()
        pass

    # ...
    PyBullet_Robot_Cls.Disconnect()
    
if __name__ == '__main__':
    main()