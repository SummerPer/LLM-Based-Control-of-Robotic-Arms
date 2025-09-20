import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from ament_index_python.packages import get_package_share_directory
import yaml, os, pathlib

def load_yaml(package_name, file_path):
    # 加载YAML文件并返回其内容
    pkg_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(pkg_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Failed to load YAML file: {absolute_file_path}, error: {e}")
        return None

def generate_launch_description():
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace', default='')
    
    # 获取包的共享目录
    deepseek_arm_control_dir = get_package_share_directory('deepseek_arm_control')
    # 注意：panda_moveit_config和panda_description是panda_gz_moveit2的子包
    # 使用绝对路径来避免包查找问题
    panda_moveit_config_dir = '/home/ubuntu22/ws_moveit2/src/panda_gz_moveit2/panda_moveit_config'
    panda_description_dir = '/home/ubuntu22/ws_moveit2/src/panda_gz_moveit2/panda_description'
    
    # 机器人描述文件路径
    robot_description_path = os.path.join(panda_description_dir, 'urdf', 'panda.urdf.xacro')
    
    # 语义描述文件路径
    robot_description_semantic_path = os.path.join(panda_moveit_config_dir, 'srdf', 'panda.srdf')
    
    # 运动学配置文件路径
    kinematics_config_path = os.path.join(deepseek_arm_control_dir, 'config', 'kinematics.yaml')
    
    # 加载运动学配置文件内容
    kinematics_config = {}
    if os.path.exists(kinematics_config_path):
        try:
            with open(kinematics_config_path, 'r') as file:
                kinematics_config = yaml.safe_load(file)
        except Exception as e:
            print(f"Failed to load kinematics config: {e}")
    
    # 控制器配置文件路径
    controllers_config_path = os.path.join(panda_moveit_config_dir, 'config', 'controllers_position.yaml')
    moveit_controller_manager_config_path = os.path.join(panda_moveit_config_dir, 'config', 'moveit_controller_manager.yaml')
    
    # 加载控制器配置文件内容
    controllers_config = {}
    if os.path.exists(controllers_config_path):
        try:
            with open(controllers_config_path, 'r') as file:
                controllers_config = yaml.safe_load(file)
        except Exception as e:
            print(f"Failed to load controllers config: {e}")
    
    # 加载MoveIt控制器管理器配置文件内容
    moveit_controller_manager_config = {}
    if os.path.exists(moveit_controller_manager_config_path):
        try:
            with open(moveit_controller_manager_config_path, 'r') as file:
                moveit_controller_manager_config = yaml.safe_load(file)
        except Exception as e:
            print(f"Failed to load moveit controller manager config: {e}")
    
    # 启动robot_state_publisher节点，发布机器人状态
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': Command(['xacro ', 'name:=panda', ' ', robot_description_path])}
        ]
    )
    
    # 创建节点启动描述，添加完整的控制器配置
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': Command(['xacro ', 'name:=panda', ' ', robot_description_path])},
            {'robot_description_semantic': open(robot_description_semantic_path, 'r').read()},
            {'robot_description_kinematics': kinematics_config},
            {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
            {'moveit_simple_controller_manager.moveit_controller_names': ['joint_trajectory_controller', 'gripper_trajectory_controller']},
            {'moveit_simple_controller_manager.default_controller_namespace': ''},
            {'controller_manager_name': 'controller_manager'},
            {'trajectory_execution.execution_duration_monitoring': False},
            {'trajectory_execution.allowed_execution_duration_scaling': 1.2},
            {'trajectory_execution.allowed_goal_duration_margin': 0.5},
            moveit_controller_manager_config,
            controllers_config
        ]
    )
    
    # 创建ros2_control_node，用于控制机器人
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers_config,
            {'use_sim_time': use_sim_time},
            {'robot_description': Command(['xacro ', 'name:=panda', ' ', robot_description_path])}
        ],
        output='screen'
    )
    
    # 启动joint_state_broadcaster控制器
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # 启动joint_trajectory_controller控制器
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # 启动hand_controller控制器
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # 创建机械臂服务节点
    arm_mover_server_node = Node(
        package='deepseek_arm_control',
        executable='arm_mover_server_node',
        name='arm_mover_server',
        output='screen',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': Command(['xacro ', 'name:=panda', ' ', robot_description_path])},
            {'robot_description_semantic': open(robot_description_semantic_path, 'r').read()},
            {'robot_description_kinematics': kinematics_config}
        ]
    )
    
    # 生成启动描述
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        move_group_node,
        arm_mover_server_node
    ])