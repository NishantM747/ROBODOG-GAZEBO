import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import xacro

world_file = os.path.join(get_package_share_directory('rover_gazebosim'), 'worlds/world.sdf')
def generate_launch_description():
    # Load robot description from URDF file
    robot_description = xacro.process_file(
        os.path.join(get_package_share_directory('rover_gazebosim'), 'urdf/robodog_urdf_visual.urdf')
    ).toxml()
    ekf_config_file = os.path.join(
        get_package_share_directory('odomshi'),
        'config',
        'ekf_config.yaml'
    )
    ukf_config_file = os.path.join(
        get_package_share_directory('odomshi'),
        'config',
        'ukf_config.yaml'
    )

    # Load configuration for parameter bridge
    config_file = os.path.join(get_package_share_directory('rover_gazebosim'), 'config', 'parameter_bridge.yaml')

    return LaunchDescription([
        # Launch arguments for GUI and initial position
        DeclareLaunchArgument('gui', default_value='true', description='Enable/Disable GUI'),
        DeclareLaunchArgument('x', default_value='0', description='Initial x position of the rover'),
        DeclareLaunchArgument('y', default_value='0', description='Initial y position of the rover'),
        DeclareLaunchArgument('z', default_value='0.5', description='Initial z position of the rover'),

        # Set environment variables for resource paths
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=os.path.join(get_package_share_directory('rover_gazebosim'))
        ),

        # Launch Ignition Gazebo with a specific world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": world_file
            }.items(),
        ),

        # Robot State Publisher for publishing the robot's URDF to the parameter server
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{"robot_description": robot_description},{'use_sim_time':True}]
        ),

        #Joint State Publisher for publishing joint states
        Node(
          package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': LaunchConfiguration('gui')},{'use_sim_time':True}]
        ),

        # Spawn the rover in Ignition Gazebo
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            name="rover_spawn",
            arguments=[
                "-string", robot_description,
                "-name", "rover",
                "-x", LaunchConfiguration("x"),
                "-y", LaunchConfiguration("y"),
                "-z", LaunchConfiguration("z"),
                
            ],
            parameters = [{'use_sim_time':True}],
        ),

        # ROS-Gazebo Bridge for parameter communication
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': config_file},{'use_sim_time':True}],
        ),

        # EKF localization node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_file,{'use_sim_time':True}],
        ),
        Node(
            package = 'odomshi',
            executable = 'relay',
            name = 'relay',
            output = 'screen',
            parameters = [{'use_sim_time':True}],
        ),
        Node(
            package = 'poc_pipeline',
            executable = 'dog_command_node',
            name = 'angle_publisher',
            output = 'screen',
            parameters = [{'use_sim_time': True}],
        ),
        Node(
            package = 'carrot_stick',
            executable = 'controller',
            name = 'controller',
            output = 'screen',
            parameters = [{'use_sim_time': True}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output = 'screen',
            parameters = [{'use_sim_time':True}],
        ),

    ])
