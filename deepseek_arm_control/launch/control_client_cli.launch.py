import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    use_simulation_mode_arg = DeclareLaunchArgument(
        'use_simulation_mode',
        default_value='false',
        description='是否使用模拟模式（false表示实际控制机械臂）'
    )
    
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='-0.18',
        description='目标x坐标'
    )
    
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.37',
        description='目标y坐标'
    )
    
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.35',
        description='目标z坐标'
    )
    
    qx_arg = DeclareLaunchArgument(
        'qx',
        default_value='0.0',
        description='目标四元数x分量'
    )
    
    qy_arg = DeclareLaunchArgument(
        'qy',
        default_value='0.0',
        description='目标四元数y分量'
    )
    
    qz_arg = DeclareLaunchArgument(
        'qz',
        default_value='0.0',
        description='目标四元数z分量'
    )
    
    qw_arg = DeclareLaunchArgument(
        'qw',
        default_value='1.0',
        description='目标四元数w分量'
    )
    
    gripper_arg = DeclareLaunchArgument(
        'gripper',
        default_value='open',
        description='夹爪状态（open或close）'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulation_mode = LaunchConfiguration('use_simulation_mode')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    qx = LaunchConfiguration('qx')
    qy = LaunchConfiguration('qy')
    qz = LaunchConfiguration('qz')
    qw = LaunchConfiguration('qw')
    gripper = LaunchConfiguration('gripper')
    
    # 创建命令行控制客户端节点
    arm_control_client_cli_node = Node(
        package='deepseek_arm_control',
        executable='arm_control_client_cli_node',
        name='arm_control_client_cli',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        arguments=[x, y, z, qx, qy, qz, qw, gripper]
    )
    
    # 生成启动描述
    return LaunchDescription([
        use_sim_time_arg,
        use_simulation_mode_arg,
        x_arg,
        y_arg,
        z_arg,
        qx_arg,
        qy_arg,
        qz_arg,
        qw_arg,
        gripper_arg,
        arm_control_client_cli_node
    ])