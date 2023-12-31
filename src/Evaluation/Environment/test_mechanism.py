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
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Get_Translation_Matrix

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the controlled mechanism.
CONST_MECHANISM_TYPE = Parameters.SMC_LEFB25_14000_0_1_Str
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Template_Industrial_Robotics')[0] + 'PyBullet_Template_Industrial_Robotics'
# The properties of the PyBullet environment.
CONST_PYBULLET_ENV_PROPERTIES = {'Enable_GUI': True, 'fps': 100, 
                                 'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance':1.3, 
                                            'Position': [0.05, -0.10, 0.06]}}

def main():
    """
    Description:
        A program to test the functionality of the designed class.
    """
    
    # Initialization of the structure of the main parameters of the mechanism.
    Mechanism_Str = CONST_MECHANISM_TYPE

    # Obtain the desired absolute position of the mechanism joint.
    theta = Mechanism_Str.Theta.Home

    # Initialization of the class to work with a mechanism object in a PyBullet environment.
    PyBullet_Mechanism_Cls = Lib.PyBullet.Core.Mechanism_Cls(Mechanism_Str, f'{CONST_PROJECT_FOLDER}/URDFs/Mechanisms/{Mechanism_Str.Name}/{Mechanism_Str.Name}.urdf', 
                                                             CONST_PYBULLET_ENV_PROPERTIES)

    # Reset the absolute position of the mechanism joint to the 'Individual'.
    PyBullet_Mechanism_Cls.Reset('Individual', theta)

    # Get the homogeneous transformation matrix of the mechanism end-effector (shuttle).
    T_Slider_new = Get_Translation_Matrix(Mechanism_Str.Theta.Axis, 
                                          theta) @ Mechanism_Str.T.Slider
    T = Mechanism_Str.T.Base @ T_Slider_new @ Mechanism_Str.T.Shuttle

    # Add a viewpoint with the correct transformation to the end-effector of the structure.
    PyBullet_Mechanism_Cls.Add_External_Object(f'{CONST_PROJECT_FOLDER}/URDFs/Viewpoint/Viewpoint.urdf', T, None, 
                                               0.5, True, False)
    
    # The physical simulation is in progress.
    while PyBullet_Mechanism_Cls.is_connected == True:
        PyBullet_Mechanism_Cls.Step()

    # Disconnect the created environment from a physical server.
    PyBullet_Mechanism_Cls.Disconnect()
    
if __name__ == '__main__':
    main()
