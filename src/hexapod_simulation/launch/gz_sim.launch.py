import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('hexapod_simulation')
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    gazebo_node = Node(
        package='gazebo_ros',
        executable='gzserver',
        name='gzserver',
        output='screen',
        arguments=['-s', 'libgazebo_ros_factory.so'],
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'hexapod',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_node,
        spawn_entity_node
    ])
