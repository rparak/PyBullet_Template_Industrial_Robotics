# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# PyBullet (Real-Time Physics Simulation) [pip3 install pybullet]
import pybullet as pb
# Custom Lib.:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot
#   ../Lib/Parameters/Mechanism
import Lib.Parameters.Mechanism
#   ../Lib/Trajectory/Utilities
import Lib.Trajectory.Utilities

class Mechanism_Cls(object):
    def __init__(self) -> None:
        # Get the FPS (Frames Per Seconds) value ...
        self.__fps = 100.0

        # Initialization of the class to generate trajectory.
        self.__Trapezoidal_Cls = Lib.Trajectory.Utilities.Trapezoidal_Profile_Cls(delta_time=1.0/self.__fps)
    
    @property
    def Theta_0(self) -> tp.List[float]:
        pass
    
    @property
    def Theta(self) -> tp.List[float]:
        pass
    
    @property
    def T_EE(self) -> tp.List[tp.List[float]]:
        pass

    def Reset(self, mode: str, theta: tp.List[float] = None) -> None:
        pass

    def Set_Absolute_Joint_Position(self, theta: tp.List[float], t_0: float, t_1: float) -> bool:
        pass

class Robot_Cls(object):
    def __init__(self) -> None:
        # Get the FPS (Frames Per Seconds) value ...
        self.__fps = 100.0

        # Initialization of the class to generate trajectory.
        self.__Polynomial_Cls = Lib.Trajectory.Utilities.Polynomial_Profile_Cls(delta_time=1.0/self.__fps)
    
    @property
    def Theta_0(self) -> tp.List[float]:
        pass
    
    @property
    def Theta(self) -> tp.List[float]:
        pass
    
    @property
    def T_EE(self) -> tp.List[tp.List[float]]:
        pass

    def Reset(self, mode: str, theta: tp.List[float] = None) -> None:
        pass

    def Set_Absolute_Joint_Position(self, theta: tp.List[float], t_0: float, t_1: float) -> bool:
        pass