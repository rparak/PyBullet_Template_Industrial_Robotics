"""
## =========================================================================== ## 
MIT License
Copyright (c) 2023 Roman Parak
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
## =========================================================================== ## 
Author   : Roman Parak
Email    : Roman.Parak@outlook.com
Github   : https://github.com/rparak
File Name: Mechanism.py
## =========================================================================== ## 
"""

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
    #   Identification of the axis direction.
    #       Note: (+1) - Positive, (-1) - Negative
    #       Unit [int]
    Direction: int = 0

@dataclass
class T_Parameters_Str:
    """
    Description:
        The auxiliary structure of the homogeneous transformation matrix {T} parameters.
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
SMC_LEFB25_14000_0_1_Str.Theta.Direction = 1

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
SMC_LEFB25_14000_0_2_Str.Theta.Direction = 1
    
