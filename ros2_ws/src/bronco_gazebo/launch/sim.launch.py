"""Start the Bronco simulation.

    ros2 launch bronco_gazebo sim.launch.py
    ros2 launch bronco_gazebo sim.launch.py rviz:=false
    ros2 launch bronco_gazebo sim.launch.py world:=/path/to/other.sdf

A launch file is a Python script that describes a set of nodes (programs) to
start together, with their parameters. This one starts five things:

  1. Gazebo, loading our world
  2. robot_state_publisher: reads the URDF, publishes it on /robot_description,
     and turns /joint_states into TF frames (wheel positions, sensor frames)
  3. create: spawns the robot from /robot_description into Gazebo
  4. parameter_bridge: connects Gazebo topics to ROS topics (config/bridge.yaml)
  5. RViz, to visualize what the robot sees
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # FindPackageShare finds install/<pkg>/share/<pkg>, where CMake installed our files
    description_share = FindPackageShare('bronco_description')
    gazebo_share = FindPackageShare('bronco_gazebo')

    urdf_file = PathJoinSubstitution([description_share, 'urdf', 'bronco.urdf.xacro'])
    rviz_file = PathJoinSubstitution([description_share, 'rviz', 'bronco.rviz'])
    bridge_file = PathJoinSubstitution([gazebo_share, 'config', 'bridge.yaml'])
    default_world = PathJoinSubstitution([gazebo_share, 'worlds', 'test_course.sdf'])

    # Launch arguments: options you can set from the command line (name:=value)
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world, description='SDF world file to load')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='Open RViz')

    # Run xacro on the model file at launch time to produce plain URDF text
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # 1. Gazebo. "-r" = start running immediately instead of paused.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    # 2. robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,   # follow Gazebo's /clock, not the wall clock
        }],
        output='screen',
    )

    # 3. Spawn the robot, slightly above the ground so it drops onto its wheels
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'bronco', '-z', '0.1'],
        output='screen',
    )

    # 4. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_file, 'use_sim_time': True}],
        output='screen',
    )

    # 5. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        declare_world,
        declare_rviz,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz_node,
    ])
