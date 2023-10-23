# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# PyBullet (Real-Time Physics Simulation) [pip3 install pybullet]
import pybullet as pb
import pybullet_data
# Custom Lib.:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot
#   ../Lib/Parameters/Mechanism
import Lib.Parameters.Mechanism
#   ../Lib/Trajectory/Utilities
import Lib.Trajectory.Utilities

"""
Description:
    Initialization of constants.
"""
# Gravitational Constant.
CONST_GRAVITY = 9.81

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
    def __init__(self, urdf_file_path: str, fps: int) -> None:
        # Time step.
        self.__delta_time = 1.0/np.float64(fps)

        # Initialization of the class to generate trajectory.
        self.__Polynomial_Cls = Lib.Trajectory.Utilities.Polynomial_Profile_Cls(delta_time=self.__delta_time)

        # ...
        self.__Set_Env_Parameters()
    
    def __Set_Env_Parameters(self):
        # ...
        pb.connect(pb.GUI)
        pb.setTimeStep(self.__delta_time)
        pb.resetSimulation()
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0.0, 0.0, -CONST_GRAVITY)

    @property
    def is_connected(self) -> bool:
        return pb.isConnected()
    
    @property
    def Theta_0(self) -> tp.List[float]:
        pass
    
    @property
    def Theta(self) -> tp.List[float]:
        pass
    
    @property
    def T_EE(self) -> tp.List[tp.List[float]]:
        pass

    @staticmethod
    def Step() -> None:
        pb.stepSimulation()

    def Disconnect(self):
        if self.is_connected == True:
            pb.disconnect()

    def Reset(self, mode: str, theta: tp.List[float] = None) -> None:
        pass

    def Set_Absolute_Joint_Position(self, theta: tp.List[float], t_0: float, t_1: float) -> bool:
        pass