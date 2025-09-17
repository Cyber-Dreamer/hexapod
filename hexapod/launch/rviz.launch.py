
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod')
    rviz_config = os.path.join(pkg_share, 'rviz', 'view.rviz')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'power_saving': True}]  # Example power setting
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
