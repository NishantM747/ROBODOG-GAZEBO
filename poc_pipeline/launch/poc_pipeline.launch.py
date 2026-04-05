from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='poc_pipeline',
            executable='dog_command_node',
            name='dog_command_node',
            output='screen'
        ),

        Node(
            package='poc_pipeline',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            output='screen'
        )

    ])

