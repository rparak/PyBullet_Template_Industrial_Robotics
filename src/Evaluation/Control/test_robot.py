# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../..')
# OS (Operating system interfaces)
import os
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/PyBullet.Core
import Lib.PyBullet.Core
#   ...
import Lib.Kinematics.Core as Kinematics

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_120_Str
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Template')[0] + 'PyBullet_Template'
# ...
#   Note:
#       ABB .. f'{CONST_PROJECT_FOLDER}/URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf'
CONST_PYBULLET_ENV_PROPERTIES = {'Enable_GUI': 0, 'fps': 100, 
                                 'External_Base': None,
                                 'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance':1.3, 
                                            'Position': [0.05, -0.10, 0.06]}}

def main():
    """
    Description:
        ...
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # ..
    PyBullet_Robot_Cls = Lib.PyBullet.Core.Robot_Cls(Robot_Str, f'{CONST_PROJECT_FOLDER}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}.urdf', 
                                                     CONST_PYBULLET_ENV_PROPERTIES)
    
    # ...
    PyBullet_Robot_Cls.Reset('Zero')

    # ...
    theta = Robot_Str.Theta.Home
    while PyBullet_Robot_Cls.is_connected == True:
        # ...
        T = Kinematics.Forward_Kinematics(theta, 'Fast', Robot_Str)[1]
        # ..
        PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Primitives/Sphere/Sphere.urdf', T, [0.0, 1.0, 0.0, 0.25], 
                                               0.025, True, False)
        PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Viewpoint/Viewpoint.urdf', T, None, 
                                               0.5, True, False)
            
        in_position = PyBullet_Robot_Cls.Set_Absolute_Joint_Position(theta, 100.0, 0.0, 2.0)
        if in_position == True:
            # ...
            theta = Robot_Str.Theta.Home if (theta == Robot_Str.Theta.Zero).all() else Robot_Str.Theta.Zero

            # ..
            PyBullet_Robot_Cls.Remove_All_External_Objects()
            in_position = False

    # ...
    PyBullet_Robot_Cls.Disconnect()
    
if __name__ == '__main__':
    main()