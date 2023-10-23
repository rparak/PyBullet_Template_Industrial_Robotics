# Numpy (Array computing) [pip3 install numpy]tp
import numpy as np
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls

@dataclass
class Theta_Parameters_Str(object):
    """
    Description:
        The auxiliary structure of the joint (theta) parameters.

        Note:
            Private structure.
    """

    # Zero absolute position of each joint.
    #   Unit [float]
    Zero: float = 0.0
    # Home absolute position of each joint.
    #   Unit [float]
    Home: float = 0.0
    # Limits of absolute joint position in radians and meters.
    #   Unit [Vector<float>]
    Limit: tp.List[float] = field(default_factory=list)
    # Other parameters of the object structure.
    #   The name of the joint.
    #       Unit [string]
    Name: str = ''
    #   Identification of the type of joint.
    #       Note: R - Revolute, P - Prismatic
    #       Unit [string]
    Type: str = ''
    #   Identification of the axis of the absolute position of the joint. 
    #       Note: 'X', 'Y', 'Z'
    #       Unit [string]
    Axis: str = ''

@dataclass
class T_Parameters_Str:
    """
    Description:
        The auxiliary structure of the homogeneous transformation matrix {T} parameters.

        Note:
            Private structure.
    """

    # Homogeneous transformation matrix of the base.
    #   Unit [Matrix<float>]
    Base: tp.List[tp.List[float]] = field(default_factory=list)
    # Homogeneous transformation matrix of the slider position.
    #   Unit [Matrix<float>]
    Slider: tp.List[tp.List[float]] = field(default_factory=list)
    # Offset in Z axis of the mechanism slider. Slider extended 
    # by an additional Shuttle.
    #   Unit [float]
    Shuttle: tp.List[tp.List[float]] = field(default_factory=list)

@dataclass
class Mechanism_Parameters_Str:
    """
    Description:
        The structure of the main parameters of the mechanism.

    Initialization of the Class (structure):
        Input:
            (1) name [string]: Name of the mechanism structure.

    Example:
        Initialization:
            Cls = Mechanism_Parameters_Str(name)
            Cls.Name = ...
            ...
            Cls.T = ..
    """

    # Name of the mechanism structure.
    #   Unit [string]
    Name: str = ''
    # Identification number.
    #   Unit [int]
    Id: int = 0
    # Absolute joint position (theta) parameters.
    #   Unit [Theta_Parameters_Str(object)]
    Theta: Theta_Parameters_Str = field(default_factory=Theta_Parameters_Str)
    # Homogeneous transformation matrix (T) parameters.
    #   Unit [T_Parameters_Str(object)]
    T: T_Parameters_Str = field(default_factory=T_Parameters_Str)

"""
Mechanism Type - SMC LEFB25UNZS 14000C (ID = 1):
    Absolute Joint Position:
        Joint L: [0.0, 1.4] [m]
"""
SMC_LEFB25_14000_0_1_Str = Mechanism_Parameters_Str(Name='SMC_LEFB25_14000', Id=1)
# Homogeneous transformation matrix of the base.
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
SMC_LEFB25_14000_0_1_Str.T.Base = HTM_Cls(None, np.float64)
# Homogeneous transformation matrix of the slider position.
#       [[1.0, 0.0, 0.0,      0.0],
#        [0.0, 1.0, 0.0,      0.0],
#        [0.0, 0.0, 1.0, 0.168499],
#        [0.0, 0.0, 0.0,      1.0]]
SMC_LEFB25_14000_0_1_Str.T.Slider = HTM_Cls(None, np.float64).Translation([0.0, 0.0, 0.168499])
SMC_LEFB25_14000_0_1_Str.T.Shuttle = HTM_Cls(None, np.float64).Translation([0.0, 0.0, 0.019999])
# Zero/Home absolute position of the joint.
SMC_LEFB25_14000_0_1_Str.Theta.Zero = 0.0
SMC_LEFB25_14000_0_1_Str.Theta.Home = 0.7
# Limits of absolute joint position.
SMC_LEFB25_14000_0_1_Str.Theta.Limit = np.array([0.0, 1.4], dtype=np.float64)
# Other parameters of the robot structure.
SMC_LEFB25_14000_0_1_Str.Theta.Name = f'Joint_L_{SMC_LEFB25_14000_0_1_Str.Name}_ID_{SMC_LEFB25_14000_0_1_Str.Id:03}'
SMC_LEFB25_14000_0_1_Str.Theta.Type = 'P'
SMC_LEFB25_14000_0_1_Str.Theta.Axis = 'Y'

"""
Mechanism Type - SMC LEFB25UNZS 14000C (ID = 2):
    Absolute Joint Position:
        Joint L: [0.0, 1.4] [m]
"""
SMC_LEFB25_14000_0_2_Str = Mechanism_Parameters_Str(Name='SMC_LEFB25_14000', Id=2)
# Homogeneous transformation matrix of the base.
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
SMC_LEFB25_14000_0_2_Str.T.Base = HTM_Cls(None, np.float64)
# Homogeneous transformation matrix of the slider position.
#       [[1.0, 0.0, 0.0,      0.0],
#        [0.0, 1.0, 0.0,      0.0],
#        [0.0, 0.0, 1.0, 0.168499],
#        [0.0, 0.0, 0.0,      1.0]]
SMC_LEFB25_14000_0_2_Str.T.Slider = HTM_Cls(None, np.float64).Translation([0.0, 0.0, 0.168499])
SMC_LEFB25_14000_0_2_Str.T.Shuttle = HTM_Cls(None, np.float64).Translation([0.0, 0.0, 0.019999])
# Zero/Home absolute position of the joint.
SMC_LEFB25_14000_0_2_Str.Theta.Zero = 0.0
SMC_LEFB25_14000_0_2_Str.Theta.Home = 0.7
# Limits of absolute joint position.
SMC_LEFB25_14000_0_2_Str.Theta.Limit = np.array([0.0, 1.4], dtype=np.float64)
# Other parameters of the robot structure.
SMC_LEFB25_14000_0_2_Str.Theta.Name = f'Joint_L_{SMC_LEFB25_14000_0_2_Str.Name}_ID_{SMC_LEFB25_14000_0_2_Str.Id:03}'
SMC_LEFB25_14000_0_2_Str.Theta.Type = 'P'
SMC_LEFB25_14000_0_2_Str.Theta.Axis = 'Y'
    