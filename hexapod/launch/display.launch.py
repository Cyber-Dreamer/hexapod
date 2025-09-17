
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
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'view.rviz')]
        )
    ])

# Allow running this file directly for testing
if __name__ == '__main__':
    import launch
    import launch_ros
    import sys
    ld = generate_launch_description()
    launch_service = launch.LaunchService(argv=sys.argv[1:])
    launch_service.include_launch_description(ld)
    sys.exit(launch_service.run())
