from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),
        # Add your main hexapod node(s) here
        Node(
            package='hexapod',
            executable='hexapod_joint_slider',
            name='hexapod_joint_slider',
            output='screen',
        ),
    ])
