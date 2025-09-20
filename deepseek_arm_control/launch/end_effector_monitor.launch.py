from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='deepseek_arm_control',
            executable='end_effector_monitor_node',
            name='end_effector_monitor',
            output='screen',
            parameters=[
                {'target_frame': 'panda_hand_tcp'},
                {'source_frame': 'panda_link0'},
                {'update_frequency': 20.0},
                {'pose_topic_name': '/end_effector_pose'},
                {'use_sim_time': True}
            ]
        )
    ])