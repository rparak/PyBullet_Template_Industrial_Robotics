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
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls, Get_Translation_Matrix
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core as Kinematics

"""
Description:
    Initialization of constants.
"""
# Gravitational Constant.
CONST_GRAVITY = 9.81

class Mechanism_Cls(object):
    def __init__(self, Mechanism_Parameters_Str: Lib.Parameters.Mechanism.Mechanism_Parameters_Str, urdf_file_path: str, properties: tp.Dict) -> None:
        # ...
        self.__Mechanism_Parameters_Str = Mechanism_Parameters_Str

        # Time step.
        self.__delta_time = 1.0/np.float64(properties['fps'])

        # Initialization of the class to generate trajectory.
        self.__Trapezoidal_Cls = Lib.Trajectory.Utilities.Trapezoidal_Profile_Cls(delta_time=self.__delta_time)

        # ...
        self.__Set_Env_Parameters(properties['Enable_GUI'], properties['Camera'])

        # ...
        self.__external_object = []

        # ...
        p = self.__Mechanism_Parameters_Str.T.Base.p.all(); q = self.__Mechanism_Parameters_Str.T.Base.Get_Rotation('QUATERNION')

        # ...
        self.__mechanism_id = pb.loadURDF(urdf_file_path, p, [q.x, q.y, q.z, q.w], useFixedBase=True, 
                                          flags=pb.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

        # ...
        self.__theta_index = 0
        for i in range(pb.getNumJoints(self.__mechanism_id)):
            info = pb.getJointInfo(self.__mechanism_id , i)
            if info[2] in [pb.JOINT_REVOLUTE, pb.JOINT_PRISMATIC]:
                self.__theta_index = i
                break

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
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, enable_gui)
        pb.configureDebugVisualizer(pb.COV_ENABLE_MOUSE_PICKING, 0)

        # ...
        plane_id = pb.loadURDF('/../../../URDFs/Primitives/Plane/Plane.urdf', globalScaling=0.20, useMaximalCoordinates=True, useFixedBase=True)

        # ...
        pb.changeVisualShape(plane_id, -1, textureUniqueId=pb.loadTexture('/../../../Textures/Plane.png'))
        pb.changeVisualShape(plane_id, -1, rgbaColor=[0.55, 0.55, 0.55, 0.95])

    @property
    def is_connected(self) -> bool:
        return pb.isConnected()
    
    @property
    def Theta_0(self) -> float:
        return self.__Mechanism_Parameters_Str.Theta.Zero
    
    @property
    def Theta(self) -> float:
        return pb.getJointState(self.__mechanism_id, self.__theta_index)[0]
    
    @property
    def T_EE(self) -> tp.List[tp.List[float]]:
        # Get the actual homogenous transformation matrix of the mechanism slider.
        T_Slider_new = Get_Translation_Matrix(self.__Mechanism_Parameters_Str.Theta.Axis, 
                                              self.Theta) @ self.__Mechanism_Parameters_Str.T.Slider
        
        return self.__Mechanism_Parameters_Str.T.Base @ T_Slider_new @ self.__Mechanism_Parameters_Str.T.Shuttle

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

    def Add_External_Object(self, urdf_file_path: str, T: HTM_Cls, rgba: tp.Union[None, tp.List[float]], scale: float, 
                            fixed: bool, enable_collision: bool):
        # ...
        p = T.p.all(); q = T.Get_Rotation('QUATERNION')

        # ...
        object_id = pb.loadURDF(urdf_file_path, p, [q.x, q.y, q.z, q.w], globalScaling=scale, useMaximalCoordinates=False, 
                                useFixedBase=fixed)

        if rgba is not None:
            pb.changeVisualShape(object_id, linkIndex=-1, rgbaColor=rgba)

        # ...
        if enable_collision == False:
            pb.setCollisionFilterGroupMask(object_id, -1, 0, 0)

        self.__external_object.append(object_id)

    def Remove_All_External_Objects(self):
        # Remove the loaded URDF model
        for _, external_obj in enumerate(self.__external_object):
            pb.removeBody(external_obj)

    def Reset(self, mode: str, theta: tp.List[float] = None) -> None:
        try:
            assert mode in ['Zero', 'Home', 'Individual']

            if mode == 'Individual':
                theta_internal = theta
            else:
                theta_internal = self.Theta_0 if mode == 'Zero' else self.__Mechanism_Parameters_Str.Theta.Home

            if self.__Mechanism_Parameters_Str.Theta.Limit[0] <= theta_internal <= self.__Mechanism_Parameters_Str.Theta.Limit[1]:
                # ...
                pb.resetJointState(self.__mechanism_id, self.__theta_index, theta_internal) 
            else:
                print(f'[WARNING] The desired input joint {theta_internal} is out of limit.')
                return False
            
            return True

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect reset mode selected. The selected mode must be chosen from the three options (Zero, Home, Individual).')

    def Set_Absolute_Joint_Position(self, theta: tp.List[float], force: float, t_0: float, t_1: float) -> bool:
        # targetVelocity = ...
        # pb.VELOCITY_CONTROL
        # https://dirkmittler.homeip.net/blend4web_ce/uranium/bullet/docs/pybullet_quickstartguide.pdf
        try:
            assert isinstance(theta, float)

            # Generation of position trajectories from input parameters.
            (theta_arr, _, _) = self.__Trapezoidal_Cls.Generate(self.Theta, theta, 0.0, 0.0, 
                                                                t_0, t_1)

            for _, theta_arr_i in enumerate(np.array(theta_arr, dtype=np.float64).T):
                if self.__Mechanism_Parameters_Str.Theta.Limit[0] <= theta_arr_i <= self.__Mechanism_Parameters_Str.Theta.Limit[1]:
                    # ...
                    pb.setJointMotorControl2(self.__mechanism_id, self.__theta_index, pb.POSITION_CONTROL, targetPosition=theta_arr_i, 
                                             force=force)
                else:
                    print(f'[WARNING] The desired input joint {theta_arr_i} is out of limit.')
                    return False

                # ...
                self.Step()

                # ...
                time.sleep(self.__delta_time)

            return True
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect value type in the input variable theta. The input variable must be of type float.')

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
        self.__external_object = []

        # ...
        p = self.__Robot_Parameters_Str.T.Base.p.all(); q = self.__Robot_Parameters_Str.T.Base.Get_Rotation('QUATERNION')
        
        if properties['External_Base'] != None:
            base_id = pb.loadURDF(properties['External_Base'], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 
                                 useFixedBase=True)
            
            # disable all collisions with the base_id
            pb.setCollisionFilterGroupMask(base_id, -1, 0, 0)

            # ...
            self.__robot_id = pb.loadURDF(urdf_file_path, p, [q.x, q.y, q.z, q.w], useFixedBase=True, 
                                        flags=pb.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
            
            # ...
            pb.setCollisionFilterPair(self.__robot_id, base_id, -1,-1, 1)
        else:
            # ...
            self.__robot_id = pb.loadURDF(urdf_file_path, p, [q.x, q.y, q.z, q.w], useFixedBase=True, 
                                        flags=pb.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
        # ...
        self.__theta_index = []
        for i in range(pb.getNumJoints(self.__robot_id)):
            info = pb.getJointInfo(self.__robot_id , i)
            if info[2] in [pb.JOINT_REVOLUTE, pb.JOINT_PRISMATIC]:
                self.__theta_index.append(i)

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
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, enable_gui)
        pb.configureDebugVisualizer(pb.COV_ENABLE_MOUSE_PICKING, 0)

        # ...
        plane_id = pb.loadURDF('/../../../URDFs/Primitives/Plane/Plane.urdf', globalScaling=0.20, useMaximalCoordinates=True, useFixedBase=True)

        # ...
        pb.changeVisualShape(plane_id, -1, textureUniqueId=pb.loadTexture('/../../../Textures/Plane.png'))
        pb.changeVisualShape(plane_id, -1, rgbaColor=[0.55, 0.55, 0.55, 0.95])

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
        for i, th_index in enumerate(self.__theta_index):
            theta_out[i] = pb.getJointState(self.__robot_id, th_index)[0]

        return theta_out
    
    @property
    def T_EE(self) -> tp.List[tp.List[float]]:
        return Kinematics.Forward_Kinematics(self.Theta, 'Fast', self.__Robot_Parameters_Str)[1]

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

    def Add_External_Object(self, urdf_file_path: str, T: HTM_Cls, rgba: tp.Union[None, tp.List[float]], scale: float, 
                            fixed: bool, enable_collision: bool):
        # ...
        p = T.p.all(); q = T.Get_Rotation('QUATERNION')

        # ...
        object_id = pb.loadURDF(urdf_file_path, p, [q.x, q.y, q.z, q.w], globalScaling=scale, useMaximalCoordinates=False, 
                                useFixedBase=fixed)

        if rgba is not None:
            pb.changeVisualShape(object_id, linkIndex=-1, rgbaColor=rgba)

        # ...
        if enable_collision == False:
            pb.setCollisionFilterGroupMask(object_id, -1, 0, 0)

        self.__external_object.append(object_id)

    def Remove_All_External_Objects(self):
        # Remove the loaded URDF model
        for _, external_obj in enumerate(self.__external_object):
            pb.removeBody(external_obj)

    def Reset(self, mode: str, theta: tp.List[float] = None) -> None:
        try:
            assert mode in ['Zero', 'Home', 'Individual']

            if mode == 'Individual':
                theta_internal = theta
            else:
                theta_internal = self.Theta_0 if mode == 'Zero' else self.__Robot_Parameters_Str.Theta.Home

            for i, (th_i, th_i_limit, th_index) in enumerate(zip(theta_internal, self.__Robot_Parameters_Str.Theta.Limit, 
                                                                 self.__theta_index)):

                if th_i_limit[0] <= th_i <= th_i_limit[1]:
                    # ...
                    pb.resetJointState(self.__robot_id, th_index, th_i) 

                else:
                    print(f'[WARNING] The desired input joint {th_i} in index {i} is out of limit.')
                    return False
                
            return True

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect reset mode selected. The selected mode must be chosen from the three options (Zero, Home, Individual).')

    def Set_Absolute_Joint_Position(self, theta: tp.List[float], force: float, t_0: float, t_1: float) -> bool:
        # targetVelocity = ...
        # pb.VELOCITY_CONTROL
        # https://dirkmittler.homeip.net/blend4web_ce/uranium/bullet/docs/pybullet_quickstartguide.pdf
        try:
            assert self.__Robot_Parameters_Str.Theta.Zero.size == theta.size

            # Generation of multi-axis position trajectories from input parameters.
            theta_arr = []
            for _, (th_actual, th_desired) in enumerate(zip(self.Theta, theta)):
                (theta_arr_i, _, _) = self.__Polynomial_Cls.Generate(th_actual, th_desired, 0.0, 0.0, 0.0, 0.0,
                                                                     t_0, t_1)
                theta_arr.append(theta_arr_i)

            for _, theta_arr_i in enumerate(np.array(theta_arr, dtype=np.float64).T):
                for i, (th_i, th_i_limit, th_index) in enumerate(zip(theta_arr_i, self.__Robot_Parameters_Str.Theta.Limit, 
                                                                     self.__theta_index)): 
                    if th_i_limit[0] <= th_i <= th_i_limit[1]:
                        # ...
                        pb.setJointMotorControl2(self.__robot_id, th_index, pb.POSITION_CONTROL, targetPosition=th_i, 
                                                 force=force)
                    else:
                        print(f'[WARNING] The desired input joint {th_i} in index {i} is out of limit.')
                        return False

                # ...
                self.Step()

                # ...
                time.sleep(self.__delta_time)

            return True
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] Incorrect number of values in the input variable theta. The input variable "theta" must contain {self.__Robot_Parameters_Str.Theta.Zero.size} values.')