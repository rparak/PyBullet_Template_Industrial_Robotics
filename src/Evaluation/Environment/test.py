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
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str
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

    PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Viewpoint/Viewpoint.urdf', PyBullet_Robot_Cls.T_EE, None, 
                                           0.5, False)
    
    """
    PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Primitives/Cube/Cube.urdf', PyBullet_Robot_Cls.T_EE, [1.0, 0.0, 1.0, 0.5], 
                                           0.1, False)
    """

    # ...
    while PyBullet_Robot_Cls.is_connected == True:
        #_ = PyBullet_Robot_Cls.Set_Absolute_Joint_Position(Robot_Str.Theta.Zero, 0.0, 5.0)
        pass

    # ...
    PyBullet_Robot_Cls.Disconnect()
    
if __name__ == '__main__':
    main()