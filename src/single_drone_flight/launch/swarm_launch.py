from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros_com',
            executable='px4_ros_com_node',
            namespace='uav1',
        ),
        Node(
            package='px4_ros_com',
            executable='px4_ros_com_node',
            namespace='uav2',
        ),
        Node(
            package='swarm_drone',
            executable='follower_node',
        )
    ])