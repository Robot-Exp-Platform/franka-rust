class LoadState:
    m : float
    x : list[float]
    i : list[float]
    ...
    
class Pose:
    """
    # Pose
    The same position can be represented in different ways. Different description methods are uniformly expressed as the `Pose` object  
    同一个位置可以用不同的方式表示, 将不同的描述方式统一表述为 `Pose` 对象
    
    ## constructor 构造函数
    
    ```python
    pose = Pose.Euler([x, y, z], [roll, pitch, yaw])
    pose = Pose.Quat([x, y, z], [qx, qy, qz, qw])
    pose = Pose.Homo(homo_array: list)
    pose = Pose.AxisAngle([x, y, z], [axis_x, axis_y, axis_z], angle)
    pose = Pose.Position([x, y, z])
    ```
    
    ## methods 方法
    
    ```python
    (tran, rot) = pose.euler()
    (tran, rot) = pose.quat()
    (homo_array) = pose.homo()
    (tran, axis, angle) = pose.axis_angle()
    (tran) = pose.position()
    ```
    """
    @classmethod
    def Euler(cls, tran: list[float], rot: list[float]) -> 'Pose':
        """
        Create a Pose from Euler angles. 从欧拉角创建位姿
        """
        ...
    @classmethod
    def Quat(cls, tran: list[float], rot: list[float]) -> 'Pose':
        """
        Create a Pose from Quaternion angles. 从四元数创建位姿
        """
        ...
    @classmethod
    def Homo(cls, homo_array: list[float]) -> 'Pose':
        """
        Create a Pose from Homogeneous coordinates. 从齐次坐标创建位姿
        """
        ...
    @classmethod
    def AxisAngle(cls, tran: list[float], axis: list[float], angle: float) -> 'Pose':
        """
        Create a Pose from Axis-Angle representation. 从轴角表示创建位姿
        """
        ...    
    @classmethod
    def Position(cls, tran: list[float]) -> 'Pose':
        """
        Create a Pose from Position representation. 从位置表示创建位姿
        """
        ...

    def euler(self) -> tuple[list[float], list[float]]:
        """
        Convert the pose into the Euler Angle expression 将位姿转化为欧拉角表述  
        ```
        ([x, y, z], [roll, pitch, yaw])
        ```
        Returns the Euler angles as a tuple of two lists: translation and rotation.
        """
        ...
    
    def quat(self) -> tuple[list[float], list[float]]:
        """
        Convert the pose into the Quaternion expression 将位姿转化为四元数表述  
        ```
        ([x, y, z], [qx, qy, qz, qw])
        ```  
        Returns the quaternion as a tuple of two lists: translation and rotation. 
        """
        ...
        
    def homo(self) -> list[float]:
        """
        Convert the pose into the homogeneous representation 将位姿转化为齐次表示,这里并非其次变换矩阵，而是按列存储的列表  
        ```
        list[float]
        ```
        Returns the homogeneous representation as a list.
        """
        ...
        
    def axis_angle(self) -> tuple[list[float], list[float], float]:
        """
        Convert the pose into the Axis-Angle representation 将位姿转化为轴角表述  
        ```
        ([x, y, z], [axis_x, axis_y, axis_z], angle)
        ```
        Returns the axis-angle representation as a tuple of a list for translation, a list for the axis, and a float for the angle.
        """
        ...
        
    def position(self) -> list[float]:
        """
        Convert the pose into the Position representation 将位姿转化为位置表示  
        ```
        [x, y, z]
        ```
        Returns the position as a list.
        """
        ...

class RobotState:
    ...

class Robot:
    """
    # Robot
    
    ## methods(as `RobotBehavior`) 方法
    """
    
    def version(self) -> str:
        """get the version of the robot 获取机器人版本号
        Returns:
            str: the version of the robot 机器人版本号
        """
        ...
 
    def init(self) -> None:
        """initialize the robot 初始化机器人"""
        ...

    def shutdown(self) -> None:
        """shutdown the robot 关闭机器人"""
        ...

    def enable(self) -> None:
        """enable the robot 使能机器人"""
        ...

    def disable(self) -> None:
        """disable the robot 去使能机器人"""
        ...
        
    def reset(self) -> None:
        """reset the robot 复位机器人"""
        ...
    
    def is_moving(self) -> bool:
        """check if robot is moving 检查机器人是否在运动中
        Returns:
            bool: check if robot is moving 是否在运动状态
        """
        ...

    def stop(self) -> None:
        """stop the current action 停止当前动作，不可恢复"""
        ...
        
    def pause(self) -> None:
        """pause the robot 运动暂停"""
        ...

    def resume(self) -> None:
        """resume the robot 运动恢复"""
        ...

    def emergency_stop(self) -> None:
        """emergency stop the robot 紧急停止机器人"""
        ...

    def clear_emergency_stop(self) -> None:
        """clear emergency stop status 清除紧急停止状态"""
        ...
    
    def read_state(self) -> RobotState:
        """read the robot state 读取机器人状态
        Returns:
            RobotState: the robot state 机器人状态
        """
        ...
    ...

class MotionType:
    """
    # MotionType
    """
    
    @classmethod
    def Joint(cls, joint: list[float]) -> 'MotionType':
        """
        Create a joint space motion type. 创建关节空间运动类型
        """
        ...
    @classmethod
    def JointVel(cls, joint_vel: list[float]) -> 'MotionType':
        """
        Create a joint velocity motion type. 创建关节速度运动类型
        """
        ...
    @classmethod
    def Cartesian(cls, pose: Pose) -> 'MotionType':
        """
        Create a Cartesian space motion type. 创建笛卡尔空间运动类型
        """
        ...
    @classmethod
    def CartesianVel(cls, cartesian_vel: list[float]) -> 'MotionType':
        """
        Create a Cartesian velocity motion type. 创建笛卡尔速度运动类型
        """
        ...
    @classmethod
    def Position(cls, position: list[float]) -> 'MotionType':
        """
        Create a position motion type. 创建位置运动类型
        """
        ...
    @classmethod
    def PositionVel(cls, position_vel: list[float]) -> 'MotionType':
        """
        Create a position velocity motion type. 创建位置速度运动类型
        """
        ...
    @classmethod
    def Stop(cls) -> 'MotionType':
        """
        Create a stop motion type. 创建停止运动类型
        """
        ...
    ...

class ControlType:
    """
    # ControlType
    """
    
    @classmethod
    def Torque(cls, tau: list[float]) -> 'ControlType':
        """
        Create a torque control type. 创建力矩控制类型
        """
        ...
        
    @classmethod
    def Zero(cls) -> 'ControlType':
        """
        Create a zero control type. 创建零力控制类型
        """
        ...
    ...
    
class ArmState:
    """
    ArmState

    Represents the state of a robot arm, including joint positions, velocities, accelerations, torques, end-effector poses, Cartesian velocities, and load state.

    机械臂状态，包含关节位置、速度、加速度、力矩、末端位姿、笛卡尔速度和负载状态等信息。
    """
    joint: list[float] | None
    joint_vel: list[float] | None
    joint_acc: list[float] | None
    tau: list[float] | None
    pose_o_to_ee: Pose | None
    pose_ee_to_k: Pose | None
    cartesian_vel: list[float] | None
    load: LoadState | None

    def __init__(
        self,
        joint: list[float] | None = ...,
        joint_vel: list[float] | None = ...,
        joint_acc: list[float] | None = ...,
        tau: list[float] | None = ...,
        pose_o_to_ee: Pose | None = ...,
        pose_ee_to_k: Pose | None = ...,
        cartesian_vel: list[float] | None = ...,
        load: LoadState | None = ...,
    ):
        """
        Initialize ArmState.

        初始化机械臂状态。
        """
        ...
    def joint(self) -> list[float] | None:
        """
        Get the joint positions of the arm.

        获取机械臂的关节位置。
        """
        ...
    def joint_vel(self) -> list[float] | None:
        """
        Get the joint velocities of the arm.

        获取机械臂的关节速度。
        """
        ...
    def joint_acc(self) -> list[float] | None:
        """
        Get the joint accelerations of the arm.

        获取机械臂的关节加速度。
        """
        ...
    def tau(self) -> list[float] | None:
        """
        Get the joint torques of the arm.

        获取机械臂的关节力矩。
        """
        ...
    def pose_o_to_ee(self) -> Pose | None:
        """
        Get the pose of the end effector in the base frame.

        获取末端执行器在基坐标系下的位姿。
        Returns:
            Pose: the pose of the end effector in the base frame 末端执行器在基坐标系下的位姿
        """
        ...
    def pose_f_to_ee(self) -> Pose | None:
        """
        Get the pose of the end effector in the flange frame.

        获取末端执行器在法兰坐标系下的位姿。
        Returns:
            Pose: the pose of the end effector in the flange frame 末端执行器在法兰坐标系下的位姿
        """
        ...
    def pose_ee_to_kin(self) -> Pose | None:
        """
        Get the pose of the end effector in the kin frame.

        获取末端执行器在关节坐标系下的位姿。
        Returns:
            Pose: the pose of the end effector in the kin frame 末端执行器在关节坐标系下的位姿
        """
        ...
    def cartesian_val(self) -> list[float] | None:
        """
        Get the Cartesian velocity of the end effector.

        获取末端执行器在笛卡尔坐标系下的速度。
        Returns:
            list[float]: the cartesian velocity of the end effector 末端执行器在笛卡尔坐标系下的速度
        """
        ...
    def load(self) -> LoadState | None:
        """
        Get the load state of the end effector.

        获取末端执行器的负载状态。
        Returns:
            LoadState: the load state of the end effector 末端执行器的负载状态
        """
        ...

class Arm(Robot):
    """
    Arm

    Basic interface for robot arm behavior, including state query, load and coordinate system settings, and motion parameters.

    机械臂基础行为接口，包括状态查询、负载与坐标系设置、运动参数设置等。
    """
    def state(self) -> ArmState:
        """
        Get the current state of the robot arm.

        获取机械臂当前状态。
        """
        ...
    def set_load(self, load: LoadState) -> None:
        """
        Set the load state of the end effector.

        设置末端执行器的负载状态。
        """
        ...
    def set_coord(self, coord: str) -> None:
        """
        Set the coordinate system for the robot arm.

        设置机械臂的坐标系。
        """
        ...
    def with_coord(self, coord: str) -> 'Arm':
        """
        Set the coordinate system for the next motion command.

        设置下一个运动指令的坐标系。
        """
        ...
    def set_speed(self, speed: float) -> None:
        """
        Set the speed of the robot arm.

        设置机械臂的速度。
        """
        ...
    def with_speed(self, speed: float) -> 'Arm':
        """
        Set the speed for the next motion command.

        设置下一个运动指令的速度。
        """
        ...
    def with_velocity(self, joint_vel: list[float]) -> 'Arm':
        """
        Set the joint velocity for the next motion command.

        设置下一个运动指令的关节速度。
        """
        ...
    def with_acceleration(self, joint_acc: list[float]) -> 'Arm':
        """
        Set the joint acceleration for the next motion command.

        设置下一个运动指令的关节加速度。
        """
        ...
    def with_jerk(self, joint_jerk: list[float]) -> 'Arm':
        """
        Set the joint jerk for the next motion command.

        设置下一个运动指令的关节加加速度。
        """
        ...
    def with_cartesian_velocity(self, cartesian_vel: float) -> 'Arm':
        """
        Set the Cartesian velocity for the next motion command.

        设置下一个运动指令的笛卡尔速度。
        """
        ...
    def with_cartesian_acceleration(self, cartesian_acc: float) -> 'Arm':
        """
        Set the Cartesian acceleration for the next motion command.

        设置下一个运动指令的笛卡尔加速度。
        """
        ...
    def with_cartesian_jerk(self, cartesian_jerk: float) -> 'Arm':
        """
        Set the Cartesian jerk for the next motion command.

        设置下一个运动指令的笛卡尔加加速度。
        """
        ...

class ArmPreplannedMotion:
    """
    ArmPreplannedMotion

    Interface for preplanned motion of the robot arm, supporting absolute/relative/inertial moves and path operations.

    机械臂预规划运动接口，支持绝对/相对/惯性移动及路径操作。
    """
    def move_to(self, target: 'MotionType') -> None:
        """
        Move to the target position.

        移动到目标位置。
        """
        ...
    def move_to_async(self, target: 'MotionType') -> None:
        """
        Move to the target position asynchronously.

        异步移动到目标位置。
        """
        ...
    def move_rel(self, target: 'MotionType') -> None:
        """
        Move to the target position relative to the current pose.

        相对当前位置移动到目标位置。
        """
        ...
    def move_rel_async(self, target: 'MotionType') -> None:
        """
        Move to the target position asynchronously in relative mode.

        以相对模式异步移动到目标位置。
        """
        ...
    def move_int(self, target: 'MotionType') -> None:
        """
        Move to the target position in inertial coordinate system.

        在惯性坐标系下移动到目标位置。
        """
        ...
    def move_int_async(self, target: 'MotionType') -> None:
        """
        Move to the target position asynchronously in inertial coordinate system.

        在惯性坐标系下异步移动到目标位置。
        """
        ...
    def move_path(self, path: list['MotionType']) -> None:
        """
        Move along a given path.

        按给定路径移动。
        """
        ...
    def move_path_async(self, path: list['MotionType']) -> None:
        """
        Move along a given path asynchronously.

        异步按给定路径移动。
        """
        ...
    def move_path_prepare(self, path: list['MotionType']) -> None:
        """
        Prepare for path motion.

        准备路径运动。
        """
        ...
    def move_path_start(self, start: 'MotionType') -> None:
        """
        Start path motion from a given start point.

        从指定起点开始路径运动。
        """
        ...

class ArmPreplannedMotionImpl:
    """
    ArmPreplannedMotionImpl

    Implementation interface for preplanned motion, providing joint and Cartesian space motion methods.

    机械臂预规划运动实现接口，提供关节空间和笛卡尔空间的运动方法。
    """
    def move_joint(self, target: list[float]) -> None:
        """
        Move in joint space to the target.

        关节空间移动到目标。
        """
        ...
    def move_joint_async(self, target: list[float]) -> None:
        """
        Move in joint space to the target asynchronously.

        关节空间异步移动到目标。
        """
        ...
    def move_cartesian(self, target: 'Pose') -> None:
        """
        Move in Cartesian space to the target pose.

        笛卡尔空间移动到目标位姿。
        """
        ...
    def move_cartesian_async(self, target: 'Pose') -> None:
        """
        Move in Cartesian space to the target pose asynchronously.

        笛卡尔空间异步移动到目标位姿。
        """
        ...

class ArmPreplannedMotionExt:
    """
    ArmPreplannedMotionExt

    Extension interface for preplanned motion, supporting relative, path, and various linear moves.

    机械臂预规划运动扩展接口，支持相对、路径及多种直线运动方式。
    """
    def move_joint_rel(self, target: list[float]) -> None:
        """
        Move relatively in joint space.

        关节空间相对移动。
        """
        ...
    def move_joint_rel_async(self, target: list[float]) -> None:
        """
        Move relatively in joint space asynchronously.

        关节空间异步相对移动。
        """
        ...
    def move_joint_path(self, path: list[list[float]]) -> None:
        """
        Move along a joint space path.

        按关节空间路径移动。
        """
        ...
    def move_cartesian_rel(self, target: 'Pose') -> None:
        """
        Move relatively in Cartesian space.

        笛卡尔空间相对移动。
        """
        ...
    def move_cartesian_rel_async(self, target: 'Pose') -> None:
        """
        Move relatively in Cartesian space asynchronously.

        笛卡尔空间异步相对移动。
        """
        ...
    def move_cartesian_int(self, target: 'Pose') -> None:
        """
        Move in inertial Cartesian space.

        惯性坐标系下笛卡尔空间移动。
        """
        ...
    def move_cartesian_int_async(self, target: 'Pose') -> None:
        """
        Move in inertial Cartesian space asynchronously.

        惯性坐标系下笛卡尔空间异步移动。
        """
        ...
    def move_cartesian_path(self, path: list['Pose']) -> None:
        """
        Move along a Cartesian space path.

        按笛卡尔空间路径移动。
        """
        ...
    def move_linear_with_euler(self, pose: list[float]) -> None:
        """
        Move linearly using Euler angles.

        以欧拉角方式直线移动。
        """
        ...
    def move_linear_with_euler_async(self, pose: list[float]) -> None:
        """
        Move linearly using Euler angles asynchronously.

        以欧拉角方式异步直线移动。
        """
        ...
    def move_linear_with_euler_rel(self, pose: list[float]) -> None:
        """
        Move linearly and relatively using Euler angles.

        以欧拉角方式相对直线移动。
        """
        ...
    def move_linear_with_euler_rel_async(self, pose: list[float]) -> None:
        """
        Move linearly and relatively using Euler angles asynchronously.

        以欧拉角方式异步相对直线移动。
        """
        ...
    def move_linear_with_euler_int(self, pose: list[float]) -> None:
        """
        Move linearly in inertial coordinates using Euler angles.

        以欧拉角方式惯性直线移动。
        """
        ...
    def move_linear_with_euler_int_async(self, pose: list[float]) -> None:
        """
        Move linearly in inertial coordinates using Euler angles asynchronously.

        以欧拉角方式惯性异步直线移动。
        """
        ...
    def move_linear_with_quat(self, target) -> None:
        """
        Move linearly using quaternion.

        以四元数方式直线移动。
        """
        ...
    def move_linear_with_quat_async(self, target) -> None:
        """
        Move linearly using quaternion asynchronously.

        以四元数方式异步直线移动。
        """
        ...
    def move_linear_with_quat_rel(self, target) -> None:
        """
        Move linearly and relatively using quaternion.

        以四元数方式相对直线移动。
        """
        ...
    def move_linear_with_quat_rel_async(self, target) -> None:
        """
        Move linearly and relatively using quaternion asynchronously.

        以四元数方式异步相对直线移动。
        """
        ...
    def move_linear_with_quat_int(self, target) -> None:
        """
        Move linearly in inertial coordinates using quaternion.

        以四元数方式惯性直线移动。
        """
        ...
    def move_linear_with_quat_int_async(self, target) -> None:
        """
        Move linearly in inertial coordinates using quaternion asynchronously.

        以四元数方式惯性异步直线移动。
        """
        ...
    def move_linear_with_homo(self, target: list[float]) -> None:
        """
        Move linearly using homogeneous matrix.

        以齐次矩阵方式直线移动。
        """
        ...
    def move_linear_with_homo_async(self, target: list[float]) -> None:
        """
        Move linearly using homogeneous matrix asynchronously.

        以齐次矩阵方式异步直线移动。
        """
        ...
    def move_linear_with_homo_rel(self, target: list[float]) -> None:
        """
        Move linearly and relatively using homogeneous matrix.

        以齐次矩阵方式相对直线移动。
        """
        ...
    def move_linear_with_homo_rel_async(self, target: list[float]) -> None:
        """
        Move linearly and relatively using homogeneous matrix asynchronously.

        以齐次矩阵方式异步相对直线移动。
        """
        ...
    def move_linear_with_homo_int(self, target: list[float]) -> None:
        """
        Move linearly in inertial coordinates using homogeneous matrix.

        以齐次矩阵方式惯性直线移动。
        """
        ...
    def move_linear_with_homo_int_async(self, target: list[float]) -> None:
        """
        Move linearly in inertial coordinates using homogeneous matrix asynchronously.

        以齐次矩阵方式惯性异步直线移动。
        """
        ...
    def move_path_prepare_from_file(self, path: str) -> None:
        """
        Prepare path motion from file.

        从文件准备路径运动。
        """
        ...
    def move_path_from_file(self, path: str) -> None:
        """
        Execute path motion from file.

        从文件执行路径运动。
        """
        ...

class ArmStreamingHandle:
    """
    ArmStreamingHandle

    Streaming handle for robot arm, provides access to last motion/control and allows setting new targets.

    机械臂流式运动句柄，提供上一个运动/控制目标的访问与新目标的设置。
    """
    def last_motion(self) -> 'MotionType':
        """
        Get the last motion target.

        获取上一个运动目标。
        """
        ...
    def move_to(self, target: 'MotionType') -> None:
        """
        Set a new motion target for streaming.

        设置新的流式运动目标。
        """
        ...
    def last_control(self) -> 'ControlType':
        """
        Get the last control target.

        获取上一个控制目标。
        """
        ...
    def control_with(self, control: 'ControlType') -> None:
        """
        Set a new control target for streaming.

        设置新的流式控制目标。
        """
        ...

class ArmStreamingMotion:
    """
    ArmStreamingMotion

    Streaming motion interface for robot arm, supports starting/stopping streaming and accessing shared targets.

    机械臂流式运动接口，支持流式运动的启动/停止及目标共享对象的访问。
    """
    def start_streaming(self) -> 'ArmStreamingHandle':
        """
        Start streaming motion.

        开始流式运动。
        """
        ...
    def end_streaming(self) -> None:
        """
        End streaming motion.

        结束流式运动。
        """
        ...
    def move_to_target(self) -> object:
        """
        Get the shared object for motion target.

        获取运动目标的共享对象。
        """
        ...
    def control_with_target(self) -> object:
        """
        Get the shared object for control target.

        获取控制目标的共享对象。
        """
        ...

class ArmStreamingMotionExt:
    """
    ArmStreamingMotionExt

    Extension interface for streaming motion, provides access to various shared targets.

    机械臂流式运动扩展接口，提供多种目标的共享对象访问。
    """
    def move_joint_target(self) -> object:
        """
        Get the shared object for joint target.

        获取关节目标的共享对象。
        """
        ...
    def move_joint_vel_target(self) -> object:
        """
        Get the shared object for joint velocity target.

        获取关节速度目标的共享对象。
        """
        ...
    def move_joint_acc_target(self) -> object:
        """
        Get the shared object for joint acceleration target.

        获取关节加速度目标的共享对象。
        """
        ...
    def move_cartesian_target(self) -> object:
        """
        Get the shared object for Cartesian target.

        获取笛卡尔目标的共享对象。
        """
        ...
    def move_cartesian_vel_target(self) -> object:
        """
        Get the shared object for Cartesian velocity target.

        获取笛卡尔速度目标的共享对象。
        """
        ...
    def move_cartesian_euler_target(self) -> object:
        """
        Get the shared object for Euler angle target.

        获取欧拉角目标的共享对象。
        """
        ...
    def move_cartesian_quat_target(self) -> object:
        """
        Get the shared object for quaternion target.

        获取四元数目标的共享对象。
        """
        ...
    def move_cartesian_homo_target(self) -> object:
        """
        Get the shared object for homogeneous matrix target.

        获取齐次矩阵目标的共享对象。
        """
        ...
    def control_tau_target(self) -> object:
        """
        Get the shared object for torque target.

        获取力矩目标的共享对象。
        """
        ...

class ArmRealtimeControl:
    """
    ArmRealtimeControl

    Real-time control interface for robot arm, supports closure-based real-time motion and control.

    机械臂实时控制接口，支持基于闭包的实时运动与控制。
    """
    def move_with_closure(self, closure: callable[['ArmState', float], tuple['MotionType', bool]]) -> None:
        """
        Real-time motion using a closure, returns (motion target, finished).

        以闭包方式实时运动，返回(运动目标, 是否结束)。
        """
        ...
    def control_with_closure(self, closure: callable[['ArmState', float], tuple['ControlType', bool]]) -> None:
        """
        Real-time control using a closure, returns (control target, finished).

        以闭包方式实时控制，返回(控制目标, 是否结束)。
        """
        ...

class ArmRealtimeControlExt:
    """
    ArmRealtimeControlExt

    Extension interface for real-time control, supports closure-based joint/Cartesian/velocity control.

    机械臂实时控制扩展接口，支持基于闭包的关节/笛卡尔/速度控制。
    """
    def move_joint_with_closure(self, closure: callable[['ArmState', float], tuple[list[float], bool]]) -> None:
        """
        Real-time joint space motion using a closure, returns (joint target, finished).

        以闭包方式实时关节空间运动，返回(关节目标, 是否结束)。
        """
        ...
    def move_joint_vel_with_closure(self, closure: callable[['ArmState', float], tuple[list[float], bool]]) -> None:
        """
        Real-time joint velocity motion using a closure, returns (joint velocity target, finished).

        以闭包方式实时关节速度运动，返回(关节速度目标, 是否结束)。
        """
        ...
    def move_cartesian_with_closure(self, closure: callable[['ArmState', float], tuple['Pose', bool]]) -> None:
        """
        Real-time Cartesian space motion using a closure, returns (pose target, finished).

        以闭包方式实时笛卡尔空间运动，返回(位姿目标, 是否结束)。
        """
        ...
    def move_cartesian_vel_with_closure(self, closure: callable[['ArmState', float], tuple[list[float], bool]]) -> None:
        """
        Real-time Cartesian velocity motion using a closure, returns (velocity target, finished).

        以闭包方式实时笛卡尔速度运动，返回(速度目标, 是否结束)。
        """
        ...
    def move_cartesian_euler_with_closure(self, closure: callable[['ArmState', float], tuple[list[float], list[float], bool]]) -> None:
        """
        Real-time Euler angle motion using a closure, returns (translation, rotation, finished).

        以闭包方式实时欧拉角运动，返回(平移, 旋转, 是否结束)。
        """
        ...
    def move_cartesian_quat_with_closure(self, closure: callable[['ArmState', float], tuple[object, bool]]) -> None:
        """
        Real-time quaternion motion using a closure, returns (quaternion target, finished).

        以闭包方式实时四元数运动，返回(四元数目标, 是否结束)。
        """
        ...
    def move_cartesian_homo_with_closure(self, closure: callable[['ArmState', float], tuple[list[float], bool]]) -> None:
        """
        Real-time homogeneous matrix motion using a closure, returns (homogeneous matrix target, finished).

        以闭包方式实时齐次矩阵运动，返回(齐次矩阵目标, 是否结束)。
        """