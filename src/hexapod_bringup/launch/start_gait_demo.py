from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    hexapod_locomotion_pkg_path = get_package_share_directory('hexapod_locomotion')
    hexapod_simulation_pkg_path = get_package_share_directory('hexapod_simulation')

    # Launch the display.launch.py file, but disable the joint_state_publisher
    # as the gait_demo will publish joint states.
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hexapod_simulation_pkg_path, 'launch', 'display.launch.py')
        ),
        launch_arguments={'use_jsp': 'false'}.items()
    )

    # Node for the Gait Demo GUI
    gait_demo_node = Node(
        package='hexapod_locomotion',
        executable='gait_demo',
        name='gait_demo_gui',
        output='screen',
        parameters=[
            {'standoff_distance': 0.25},
            {'body_height': 0.20}
        ]
    )

    return LaunchDescription([
        display_launch,
        gait_demo_node
    ])
