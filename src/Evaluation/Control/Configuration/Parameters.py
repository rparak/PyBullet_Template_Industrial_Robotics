# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics

"""
Description:
    Initialization of constants.
"""
# Initial and final time constraints.
CONST_T_0 = 0.0
CONST_T_1 = 1.0

def Get_Absolute_Joint_Positions(name: str) -> tp.Tuple[tp.List[float],
                                                        tp.List[float]]:
    """
    Description:
        A function to obtain the constraints for absolute joint positions in order to generate 
        multi-axis position trajectories.

    Args:
        (1) name [string]: Name of the robotic structure.

    Returns:
        (1) parameter [Vector<float> 2xn]: Obtained absolute joint positions (initial, final) in radians / meters.
                                            Note:
                                                Where n is the number of joints.
    """

    return {
        'Universal_Robots_UR3': (None, None),
        'ABB_IRB_120': (None, None),
        'ABB_IRB_120_L_Ax': (None, None),
        'ABB_IRB_14000_R': (None, None),
        'ABB_IRB_14000_L': (None, None),
        'EPSON_LS3_B401S': (np.array([Mathematics.Degree_To_Radian(0.0), Mathematics.Degree_To_Radian(0.0), 0.0, Mathematics.Degree_To_Radian(0.0)], 
                                     dtype = np.float64), 
                            np.array([Mathematics.Degree_To_Radian(115.0), Mathematics.Degree_To_Radian(-20.0), 0.10, Mathematics.Degree_To_Radian(15.0)],
                                     dtype = np.float64))
    }[name]