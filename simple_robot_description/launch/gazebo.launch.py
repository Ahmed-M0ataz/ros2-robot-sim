import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('simple_robot_description')
    
    # Paths
    urdf_path = os.path.join(pkg_share, 'urdf', 'simple_robot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'house_world.sdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    
    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    # Launch Configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'simple_robot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': bridge_config_path},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add actions
    ld.add_action(gzserver)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(bridge)
    
    return ld