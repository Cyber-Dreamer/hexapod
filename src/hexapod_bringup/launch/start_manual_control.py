from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory for hexapod_simulation
    hexapod_simulation_pkg_path = get_package_share_directory('hexapod_simulation')

    # 1. Launch the display.launch.py file from hexapod_simulation
    # We set 'use_jsp' to 'false' because we want to use the GUI version
    # of the joint state publisher, not the standard one.
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hexapod_simulation_pkg_path, 'launch', 'display.launch.py')
        ),
        launch_arguments={'use_jsp': 'false'}.items()
    )

    # 2. Launch the manual_control.py file which starts the joint_state_publisher_gui
    manual_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hexapod_simulation_pkg_path, 'launch', 'manual_control.py')
        )
    )

    return LaunchDescription([
        display_launch,
        manual_control_launch
    ])