import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Find the directory for your package
    pkg_path = get_package_share_directory('hexapod_description')

    # Path to your URDF file
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'hexapod.urdf')

    # Read the URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    # Robot State Publisher node
    # This node reads the URDF and publishes the robot's state to /tf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    # Joint State Publisher GUI node
    # This node provides a GUI to manually control the joints of your robot
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 node
    # This node visualizes the robot model
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'urdf_config.rviz')],
    )

    # Return the launch description
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
