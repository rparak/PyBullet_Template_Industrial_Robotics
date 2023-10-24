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
File Name: Core.py
## =========================================================================== ## 
"""

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
    """
    Description:
        A class for working with a mechanism object in a PyBullet scene.

    Initialization of the Class:
        Args:
            (1) Mechanism_Parameters_Str [Mechanism_Parameters_Str(object)]: The structure of the main parameters of the mechanism.
            (2) ...

        Example:
            Initialization:
                # Assignment of the variables.
                #   Example for the SMC LEFB25UNZS 1400C mechanism.
                Mechanism_Parameters_Str = Lib.Parameters.Mechanism.SMC_LEFB25_1400_0_1_Str
                ...

                # Initialization of the class.
                Cls = Mechanism_Cls(Mechanism_Parameters_Str, ...)

            Features:
                # Properties of the class.
                Cls.Theta_0; Cls.T_EE

                # Functions of the class.
                Cls.Set_Absolute_Joint_Position(0.0, 100.0, 0.0, 1.0)
    """
        
    def __init__(self, Mechanism_Parameters_Str: Lib.Parameters.Mechanism.Mechanism_Parameters_Str, urdf_file_path: str, properties: tp.Dict) -> None:
        # << PRIVATE >> #
        self.__Mechanism_Parameters_Str = Mechanism_Parameters_Str
        self.__external_object = []
        # Time step.
        self.__delta_time = 1.0/np.float64(properties['fps'])

        # Initialization of the class to generate trajectory.
        self.__Trapezoidal_Cls = Lib.Trajectory.Utilities.Trapezoidal_Profile_Cls(delta_time=self.__delta_time)

        # ...
        self.__Set_Env_Parameters(properties['Enable_GUI'], properties['Camera'])

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
        """
        Description:
            Get the zero (home) absolute position of the joint in radians/meter.

        Returns:
            (1) parameter [Vector<float>]: Zero (home) absolute joint position in radians / meters.
        """
                
        return self.__Mechanism_Parameters_Str.Theta.Zero
    
    @property
    def Theta(self) -> float:
        """
        Description:
            Get the absolute position of the mechanism joint.

        Returns:
            (1) parameter [Vector<float>]: Current absolute joint position in radians / meters.
        """
                
        return pb.getJointState(self.__mechanism_id, self.__theta_index)[0]
    
    @property
    def T_EE(self) -> tp.List[tp.List[float]]:
        """
        Description:
            Get the homogeneous transformation matrix of the mechanism slider.

        Returns:
            (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix of the mechanism slider.
        """
               
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
    
    def __Step(self) -> None:
        # ..
        pb.stepSimulation()

        # ..
        time.sleep(self.__delta_time)

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

    def Reset(self, mode: str, theta: tp.Union[None, float] = None) -> bool:
        """
        Description:
            Function to reset the absolute position of the mechanism joint from the selected mode.

            Note:
                The Zero/Home modes are predefined in the mechanism structure and the Individual mode is used 
                to set the individual position defined in the function input parameter.

        Args:
            (1) mode [string]: Possible modes to reset the absolute position of the joint.
                                Note:
                                    mode = 'Zero', 'Home' or 'Individual'
            (2) theta [float]: Desired absolute joint position in radians / meters.

        Returns:
            (1) parameter [bool]: The result is 'True' if the mechanism is in the desired position,
                                  and 'False' if it is not.
        """
                
        try:
            assert mode in ['Zero', 'Home', 'Individual'] and isinstance(theta, float) == True

            if mode == 'Individual':
                theta_internal = theta
            else:
                theta_internal = self.Theta_0 if mode == 'Zero' else self.__Mechanism_Parameters_Str.Theta.Home

            if self.__Mechanism_Parameters_Str.Theta.Limit[0] <= theta_internal <= self.__Mechanism_Parameters_Str.Theta.Limit[1]:
                # Reset the state (position) of the joint.
                pb.resetJointState(self.__mechanism_id, self.__theta_index, theta_internal) 
            else:
                print(f'[WARNING] The desired input joint {theta_internal} is out of limit.')
                return False
            
            return True

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            if mode not in ['Zero', 'Home', 'Individual']:
                print('[ERROR] Incorrect reset mode selected. The selected mode must be chosen from the three options (Zero, Home, Individual).')
            if isinstance(theta, float) == False:
                print('[ERROR] Incorrect value type in the input variable theta. The input variable must be of type float.')

    def Set_Absolute_Joint_Position(self, theta: float, force: float, t_0: float, t_1: float) -> bool:
        """
        Description:
            Set the absolute position of the mechanism joint.

            Note:
                To use the velocity control of the mechanism's joint, it is necessary to change the input 
                parameters of the 'setJointMotorControl2' function from position to:

                    pb.setJointMotorControl2(self.__mechanism_id, th_index, pb.VELOCITY_CONTROL, 
                                             targetVelocity=th_v_i, force=force),
                                             
                and get the velocity from trapezoidal trajectories.

        Args:
            (1) theta [float]: Desired absolute joint position in radians / meters.
            (2) force [float]: The maximum motor force used to reach the target value.
            (3) t_0 [float]: Animation start time in seconds.
            (4) t_1 [float]: Animation stop time in seconds.

        Returns:
            (1) parameter [bool]: The result is 'True' if the mechanism is in the desired position,
                                  and 'False' if it is not.
        """
                
        try:
            assert isinstance(theta, float) == True

            # Generation of position trajectories from input parameters.
            (theta_arr, _, _) = self.__Trapezoidal_Cls.Generate(self.Theta, theta, 0.0, 0.0, 
                                                                t_0, t_1)

            for _, theta_arr_i in enumerate(np.array(theta_arr, dtype=np.float64).T):
                if self.__Mechanism_Parameters_Str.Theta.Limit[0] <= theta_arr_i <= self.__Mechanism_Parameters_Str.Theta.Limit[1]:
                    # Control of the mechanism's joint positions.
                    pb.setJointMotorControl2(self.__mechanism_id, self.__theta_index, pb.POSITION_CONTROL, targetPosition=theta_arr_i, 
                                             force=force)
                else:
                    print(f'[WARNING] The desired input joint {theta_arr_i} is out of limit.')
                    return False

                # ...
                self.__Step()

            return True
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect value type in the input variable theta. The input variable must be of type float.')

class Robot_Cls(object):
    """
    Description:
        A class for working with a robotic arm object in a PyBullet scene.

    Initialization of the Class:
        Args:
            (1) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
            (2) ...

        Example:
            Initialization:
                # Assignment of the variables.
                #   Example for the ABB IRB 120 robot.
                Robot_Parameters_Str = Lib.Parameters.Robot.ABB_IRB_120_Str
                ...

                # Initialization of the class.
                Cls = Robot_Cls(Robot_Parameters_Str, ...)

            Features:
                # Properties of the class.
                Cls.Theta_0; Cls.T_EE

                # Functions of the class.
                Cls.Set_Absolute_Joint_Position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 100.0, 0.0, 1.0)
    """
        
    def __init__(self, Robot_Parameters_Str: Lib.Parameters.Robot.Robot_Parameters_Str, urdf_file_path: str, properties: tp.Dict) -> None:
        # << PRIVATE >> #
        self.__Robot_Parameters_Str = Robot_Parameters_Str
        self.__external_object = []
        # Time step.
        self.__delta_time = 1.0/np.float64(properties['fps'])

        # Initialization of the class to generate trajectory.
        self.__Polynomial_Cls = Lib.Trajectory.Utilities.Polynomial_Profile_Cls(delta_time=self.__delta_time)

        # ...
        self.__Set_Env_Parameters(properties['Enable_GUI'], properties['Camera'])

        # ...
        p = self.__Robot_Parameters_Str.T.Base.p.all(); q = self.__Robot_Parameters_Str.T.Base.Get_Rotation('QUATERNION')
    
        # ...
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
        """
        Description:
            Get the zero (home) absolute position of the joint in radians/meter.

        Returns:
            (1) parameter [Vector<float>]: Zero (home) absolute joint position in radians / meters.
        """
                
        return self.__Robot_Parameters_Str.Theta.Zero
    
    @property
    def Theta(self) -> tp.List[float]: 
        """
        Description:
            Get the absolute positions of the robot's joints.

        Returns:
            (1) parameter [Vector<float> 1xn]: Current absolute joint position in radians / meters.
                                                Note:
                                                    Where n is the number of joints.
        """
                
        theta_out = np.zeros(self.__Robot_Parameters_Str.Theta.Zero.size, 
                             dtype=np.float64)
        for i, th_index in enumerate(self.__theta_index):
            theta_out[i] = pb.getJointState(self.__robot_id, th_index)[0]

        return theta_out
    
    @property
    def T_EE(self) -> tp.List[tp.List[float]]:
        """
        Description:
            Get the homogeneous transformation matrix of the robot end-effector.

        Returns:
            (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix of the End-Effector.
        """
                
        return Kinematics.Forward_Kinematics(self.Theta, 'Fast', self.__Robot_Parameters_Str)[1]

    @property
    def Camera_Parameters(self) -> tp.Dict:
        # ...
        parameters = pb.getDebugVisualizerCamera()

        return {'Yaw': parameters[8], 'Pitch': parameters[9], 'Distance': parameters[10], 
                'Position': parameters[11]}
    
    def __Step(self) -> None:
        # ..
        pb.stepSimulation()

        # ..
        time.sleep(self.__delta_time)

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

    def Reset(self, mode: str, theta: tp.Union[None, tp.List[float]] = None) -> bool:
        """
        Description:
            Function to reset the absolute position of the robot joints from the selected mode.

            Note:
                The Zero/Home modes are predefined in the robot structure and the Individual mode is used 
                to set the individual position defined in the function input parameter.

        Args:
            (1) mode [string]: Possible modes to reset the absolute position of the joints.
                                Note:
                                    mode = 'Zero', 'Home' or 'Individual'
            (2) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters. Used only in individual 
                                           mode.
                                            Note:
                                                Where n is the number of joints.

        Returns:
            (1) parameter [bool]: The result is 'True' if the robot is in the desired position,
                                  and 'False' if it is not.
        """
                
        try:
            assert mode in ['Zero', 'Home', 'Individual'] and self.__Robot_Parameters_Str.Theta.Zero.size == theta.size

            if mode == 'Individual':
                theta_internal = theta
            else:
                theta_internal = self.Theta_0 if mode == 'Zero' else self.__Robot_Parameters_Str.Theta.Home

            for i, (th_i, th_i_limit, th_index) in enumerate(zip(theta_internal, self.__Robot_Parameters_Str.Theta.Limit, 
                                                                 self.__theta_index)):

                if th_i_limit[0] <= th_i <= th_i_limit[1]:
                    # Reset the state (position) of the joint.
                    pb.resetJointState(self.__robot_id, th_index, th_i) 

                else:
                    print(f'[WARNING] The desired input joint {th_i} in index {i} is out of limit.')
                    return False
                
            return True

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            if mode not in ['Zero', 'Home', 'Individual']:
                print('[ERROR] Incorrect reset mode selected. The selected mode must be chosen from the three options (Zero, Home, Individual).')
            if self.__Robot_Parameters_Str.Theta.Zero.size != theta.size:
                print(f'[ERROR] Incorrect number of values in the input variable theta. The input variable "theta" must contain {self.__Robot_Parameters_Str.Theta.Zero.size} values.')

    def Set_Absolute_Joint_Position(self, theta: tp.List[float], force: float, t_0: float, t_1: float) -> bool:
        """
        Description:
            Set the absolute position of the robot joints.

            Note:
                To use the velocity control of the robot's joint, it is necessary to change the input 
                parameters of the 'setJointMotorControl2' function from position to:

                    pb.setJointMotorControl2(self.__robot_id, th_index, pb.VELOCITY_CONTROL, 
                                             targetVelocity=th_v_i, force=force),

                and get the velocity from polynomial trajectories.

        Args:
            (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                            Note:
                                                Where n is the number of joints.
            (2) force [float]: The maximum motor force used to reach the target value.
            (3) t_0 [float]: Animation start time in seconds.
            (4) t_1 [float]: Animation stop time in seconds.

        Returns:
            (1) parameter [bool]: The result is 'True' if the robot is in the desired position,
                                  and 'False' if it is not.
        """
                
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
                        # Control of the robot's joint positions.
                        pb.setJointMotorControl2(self.__robot_id, th_index, pb.POSITION_CONTROL, targetPosition=th_i, 
                                                 force=force)
                    else:
                        print(f'[WARNING] The desired input joint {th_i} in index {i} is out of limit.')
                        return False

                # ...
                self.__Step()

            return True
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] Incorrect number of values in the input variable theta. The input variable "theta" must contain {self.__Robot_Parameters_Str.Theta.Zero.size} values.')