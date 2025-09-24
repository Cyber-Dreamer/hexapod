import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Find the directory for your package
    pkg_path = get_package_share_directory('hexapod_simulation')

    # Path to your URDF file
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    # Read the URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    # Declare the launch argument
    use_jsp_arg = DeclareLaunchArgument(
        'use_jsp',
        default_value='true',
        description='Whether to start the joint_state_publisher')

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    # Joint State Publisher node (non-GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_jsp'))
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'views.rviz')],
    )

    # Return the launch description
    return LaunchDescription([
        use_jsp_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])