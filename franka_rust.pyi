class PyArmState7:
    """ 7自由度机械臂状态
    """
    
    def echo() -> str:
        ...

class FrankaRobot:
    """ 
    # Franka 机器人
    """
    def __init__(ip: str):
        ...
    
    def set_collision_behavior(self,    lower_torque_thresholds_acceleration,
                                        upper_torque_thresholds_acceleration,
                                        lower_torque_thresholds_nominal,
                                        upper_torque_thresholds_nominal,
                                        lower_force_thresholds_acceleration,
                                        upper_force_thresholds_acceleration,
                                        lower_force_thresholds_nominal,
                                        upper_force_thresholds_nominal,) -> None:
        """ 
        # FrankaRobot 
        ## 设置碰撞行为
        
        Set separate torque and force boundaries for acceleration/deceleration and constant velocity movement phases.
        Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
        Forces or torques above the upper threshold are registered as collision and cause the robot to stop moving.
        
        Args:
            lower_torque_thresholds_acceleration (list): 关节加速度下限
            upper_torque_thresholds_acceleration (list): 关节加速度上限
            lower_torque_thresholds_nominal (list): 关节名义下限
            upper_torque_thresholds_nominal (list): 关节名义上限
            lower_force_thresholds_acceleration (list): 力加速度下限
            upper_force_thresholds_acceleration (list): 力加速度上限
            lower_force_thresholds_nominal (list): 力名义下限
            upper_force_thresholds_nominal (list): 力名义上限
        
        """
        ...
    
    def set_joint_impedance(self, data: list[float]) -> None:
        """ 
        # FrankaRobot
        ## 设置关节阻抗
        
        Args:
            data: list[float; 7] 关节阻抗
        """
        ...
    
    def set_cartesian_impedance(self, data: list[float]) -> None:
        """ 
        # FrankaRobot
        ## 设置笛卡尔阻抗
        
        Args:
            data: list[float; 6] 笛卡尔阻抗
        """
        ...
        
    def set_default_behavior(self) -> None:
        """ 
        # FrankaRobot
        ## 设置默认行为
        """
        ...
    
    def set_guiding_mode(self,  guiding_mode: list[float], nullspace: bool) -> None:
        """ 
        # FrankaRobot
        ## 设置引导模式
        
        Locks or unlocks guiding mode movement in (x, y, z, roll, pitch, yaw).
        
        Args:
            guiding_mode: list[float; 6] 自由度是否锁定 
            nullspace: bool 是否使用零空间
        """
        ...
        
    def set_ee_to_k(self, data: list[float]) -> None:
        """
        # FrankaRobot
        ## 设置末端执行器到基坐标系
        
        Args:
            data: list[float; 16] 末端执行器到基坐标系的变换矩阵
        """
        ...
    
    def set_ne_to_ee(self, data: list[float]) -> None:
        """ 
        # FrankaRobot
        ## 设置末端执行器到工具坐标系
        
        Args:
            data: list[float; 16] 末端执行器到工具坐标系的变换矩阵
        """
        ...
    
    def set_load(self, m_load: float, x_load: list[float], i_load: list[float]) -> None:
        """
        # FrankaRobot
        ## 设置负载
        
        The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
        This is not for setting end effector parameters, which have to be set in the administrator's interface.
        
        Args:
            m_load: float 负载质量
            x_load: list[float; 3] 负载位置
            i_load: list[float; 9] 负载惯性
        """
        ...
        
    def arm_state(self) -> PyArmState7:
        """ 
        # FrankaRobot
        ## 机械臂状态
        Returns:
            PyArmState7 机械臂状态
        """
        ...
    
    def move_joint(self, target: list[float], speed: float) -> None:
        """
        # FrankaRobot
        ## 关节空间运动
        初始位置为 [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.]
        Args:
            target: list[float; 7] 目标关节角度
            speed: float 运动速度比
        """
        ...
        
    def move_linear_with_homo(self, target: list[float], speed: float) -> None:
        """
        # FrankaRobot
        ## 基于齐次变换矩阵的笛卡尔空间运动(按列排布的齐次变换矩阵转为的向量)
        初始位置为 
        [0.999988717111279, -0.0017158199764830543, 0.0006070528432007165, 0.0,
        -0.0017159793580267732, -0.9999888667416037, 0.0002621238143371493, 0.0,
        0.0006066080065316006, -0.00026316761376355145, -0.9999997813847427, 0.0,
        0.30648819513520104, -0.0005332618190956864, 0.48708322693240846, 1.0]
    ]
        """
        
    def move_linear_with_ruler(self, target: list[float], speed: float) -> None:
        """
        # FrankaRobot
        ## 基于欧拉角的笛卡尔空间运动
        """
        
class FrankaGripper:
    """
    # Franka 机械爪
    """
    
    def homing(self) -> bool:
        """
        # Franka 机械爪
        ## 归位
        """
        ...
    
    def grasp(self, width: float, speed: float, force: float) -> bool:
        """
        # Franka 机械爪
        ## 抓取
        
        Args:
            width: float 抓取宽度,前后裕度各 5mm
            speed: float 抓取速度
            force: float 抓取力
        """
        ...
        
    def move_gripper(self, width: float, speed: float) -> bool:
        """
        # Franka 机械爪
        ## 移动爪
        Args:
            width: float 宽度
            speed: float 速度
        """
        ...
        
    def stop(self) -> bool:
        """
        # Franka 机械爪
        ## 停止
        """
        ...