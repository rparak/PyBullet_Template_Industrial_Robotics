# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# PyBullet (Real-Time Physics Simulation) [pip3 install pybullet]
import pybullet as pb
import pybullet_data
# Time (Time access and conversions)
import time
# Custom Lib.:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot
#   ../Lib/Parameters/Mechanism
import Lib.Parameters.Mechanism
#   ../Lib/Trajectory/Utilities
import Lib.Trajectory.Utilities
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls

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
    def __init__(self, Robot_Parameters_Str: Lib.Parameters.Robot.Robot_Parameters_Str, urdf_file_path: str, properties: tp.Dict) -> None:

        # ...
        self.__Robot_Parameters_Str = Robot_Parameters_Str

        # Time step.
        self.__delta_time = 1.0/np.float64(properties['fps'])

        # Initialization of the class to generate trajectory.
        self.__Polynomial_Cls = Lib.Trajectory.Utilities.Polynomial_Profile_Cls(delta_time=self.__delta_time)

        # ...
        self.__Set_Env_Parameters(properties['Enable_GUI'], properties['Camera'])

        # ...
        self.__robot_id = pb.loadURDF(urdf_file_path, [0, 0, 0], [0, 0, 0, 1], useFixedBase=True, flags=pb.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

    
    def __Set_Env_Parameters(self, enable_gui: int, camera_properties: tp.Dict):
        # ...
        pb.connect(pb.GUI, options='--background_color_red=0.0 --background_color_green=0.0 --background_color_blue=0.0')
        pb.setTimeStep(self.__delta_time)
        pb.setRealTimeSimulation(0)
        pb.resetSimulation()
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0.0, 0.0, -CONST_GRAVITY)

        # ...
        pb.resetDebugVisualizerCamera(cameraYaw=camera_properties['Yaw'], cameraPitch=camera_properties['Pitch'], cameraDistance=camera_properties['Distance'], 
                                      cameraTargetPosition=camera_properties['Position'])
        
        # ...
        pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
        pb.configureDebugVisualizer(pb.COV_ENABLE_SHADOWS, 1)
        pb.configureDebugVisualizer(pb.COV_ENABLE_PLANAR_REFLECTION, 0)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, enable_gui)
        pb.configureDebugVisualizer(pb.COV_ENABLE_MOUSE_PICKING, 0)

        # ...
        plane_id = pb.loadURDF('/../../../URDFs/Primitives/Plane/Plane.urdf', globalScaling=0.20, useMaximalCoordinates=True, useFixedBase=True)

        # ...
        pb.changeVisualShape(plane_id, -1, textureUniqueId=pb.loadTexture('/../../../Textures/Plane.png'))
        pb.changeVisualShape(plane_id, -1, rgbaColor=[0.75, 0.75, 0.75, 0.75])

    @property
    def is_connected(self) -> bool:
        return pb.isConnected()
    
    @property
    def Theta_0(self) -> tp.List[float]:
        return self.__Robot_Parameters_Str.Theta.Zero
    
    @property
    def Theta(self) -> tp.List[float]:
        theta_out = np.zeros(self.__Robot_Parameters_Str.Theta.Zero.size, 
                             dtype=np.float64)
        for i in range(pb.getNumJoints(self.__robot_id) - 1):
            theta_out[i] = pb.getJointState(self.__robot_id, i)[0]

        print(theta_out)
    
    @property
    def T_EE(self) -> tp.List[tp.List[float]]:
        ee_link = pb.getLinkState(self.__robot_id, pb.getNumJoints(self.__robot_id) - 1, computeForwardKinematics=True)
        end_effector_position, end_effector_orientation = ee_link[0], ee_link[1]

        print(end_effector_position, end_effector_orientation)


    @property
    def Camera_Parameters(self) -> tp.Dict:
        # ...
        parameters = pb.getDebugVisualizerCamera()

        return {'Yaw': parameters[8], 'Pitch': parameters[9], 'Distance': parameters[10], 
                'Position': parameters[11]}
    
    @staticmethod
    def Step() -> None:
        pb.stepSimulation()

    def Disconnect(self):
        if self.is_connected == True:
            pb.disconnect()

    def Reset(self, mode: str, theta: tp.List[float] = None) -> None:
        try:
            assert mode in ['Zero', 'Home', 'Individual']

            if mode == 'Individual':
                theta_internal = theta
            else:
                theta_internal = self.Theta_0 if mode == 'Zero' else self.__Robot_Parameters_Str.Theta.Home

            for i, (th_i, th_i_limit, th_i_dir) in enumerate(zip(theta_internal, self.__Robot_Parameters_Str.Theta.Limit, self.__Robot_Parameters_Str.Theta.Direction)):

                if th_i_limit[0] <= th_i <= th_i_limit[1]:
                    # Change of axis direction in individual joints.
                    th_new = th_i * th_i_dir
                
                    # ...
                    pb.resetJointState(self.__robot_id, i, th_new) 

                else:
                    print(f'[WARNING] The desired input joint {th_i} in index {i} is out of limit.')
                    return False

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect reset mode selected. The selected mode must be chosen from the three options (Zero, Home, Individual).')

    def Set_Absolute_Joint_Position(self, theta: tp.List[float], t_0: float, t_1: float) -> bool:
        try:
            assert self.__Robot_Parameters_Str.Theta.Zero.size == theta.size

            # Generation of multi-axis position trajectories from input parameters.
            theta_arr = []
            for _, (th_actual, th_desired) in enumerate(zip(self.Theta, theta)):
                (theta_arr_i, _, _) = self.__Polynomial_Cls.Generate(th_actual, th_desired, 0.0, 0.0, 0.0, 0.0,
                                                                     t_0, t_1)
                theta_arr.append(theta_arr_i)

            for _, theta_arr_i in enumerate(np.array(theta_arr, dtype=np.float64).T):

                for i, (th_i, th_i_limit, th_i_dir) in enumerate(zip(theta_arr_i, self.__Robot_Parameters_Str.Theta.Limit, self.__Robot_Parameters_Str.Theta.Direction)): 
                    if th_i_limit[0] <= th_i <= th_i_limit[1]:
                        # Change of axis direction in individual joints.
                        th_new = th_i * th_i_dir

                        # ...
                        pb.setJointMotorControl2(self.__robot_id, i, pb.POSITION_CONTROL, targetPosition=th_new, force=100.0)
                    else:
                        print(f'[WARNING] The desired input joint {th_i} in index {i} is out of limit.')
                        return False
                    
                    # ...
                    self.Step()

            return True
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] Incorrect number of values in the input variable theta. The input variable "theta" must contain {self.__Robot_Parameters_Str.Theta.Zero.size} values.')