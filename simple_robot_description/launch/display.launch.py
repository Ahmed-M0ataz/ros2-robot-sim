import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('simple_robot_description')
    
    urdf_path = os.path.join(pkg_share, 'urdf', 'simple_robot.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'simple_robot.rviz')
    
    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    # Launch arguments
    use_gui = LaunchConfiguration('use_gui')
    
    # Declare launch arguments
    declare_use_gui = DeclareLaunchArgument(
        name='use_gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
    
    # Start joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(use_gui)
    )
    
    # Start joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui)
    )
    
    # Start robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )
    
    # Start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add actions
    ld.add_action(declare_use_gui)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    
    return ld