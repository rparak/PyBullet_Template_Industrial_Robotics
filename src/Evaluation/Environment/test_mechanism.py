# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../..')
# OS (Operating system interfaces)
import os
# Custom Lib.:
#   ../Lib/Parameters/Mechanism
import Lib.Parameters.Mechanism as Parameters
#   ../Lib/PyBullet.Core
import Lib.PyBullet.Core

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled mechanism.
CONST_MECHANISM_TYPE = Parameters.SMC_LEFB25_14000_0_1_Str
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Template')[0] + 'PyBullet_Template'
# ...
CONST_PYBULLET_ENV_PROPERTIES = {'Enable_GUI': 0, 'fps': 100, 
                                 'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance':1.3, 
                                            'Position': [0.05, -0.10, 0.06]}}

def main():
    """
    Description:
        ...
    """
    
    # Initialization of the structure of the main parameters of the mechanism.
    Mechanism_Str = CONST_MECHANISM_TYPE

    # ..
    PyBullet_Mechanism_Cls = Lib.PyBullet.Core.Mechanism_Cls(Mechanism_Str, f'{CONST_PROJECT_FOLDER}/URDFs/Mechanisms/{Mechanism_Str.Name}/{Mechanism_Str.Name}.urdf', 
                                                             CONST_PYBULLET_ENV_PROPERTIES)
    
    # ...
    PyBullet_Mechanism_Cls.Reset('Zero')

    """
    PyBullet_Mechanism_Cls.Add_External_Object('/../../../URDFs/Viewpoint/Viewpoint.urdf', PyBullet_Mechanism_Cls.T_EE, None, 
                                               0.5, True, False)

    """
    # ...
    while PyBullet_Mechanism_Cls.is_connected == True:
        _ = PyBullet_Mechanism_Cls.Set_Absolute_Joint_Position(Mechanism_Str.Theta.Home, 100.0, 0.0, 5.0)
        #pass

    # ...
    PyBullet_Mechanism_Cls.Disconnect()
    
if __name__ == '__main__':
    main()