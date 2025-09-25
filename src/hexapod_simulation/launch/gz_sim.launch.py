import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Get the package share directory
    pkg_path = get_package_share_directory('hexapod_simulation')
    
    # Correctly determine the workspace's install directory
    install_dir = os.path.join(pkg_path, '..')

    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')

    # Path to the Gazebo launch file
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    # Path to your URDF file
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    # Read the URDF file content
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        # Log the Gazebo resource path for debugging
        LogInfo(msg=['GZ_SIM_RESOURCE_PATH set to: ', install_dir]),

        # This is necessary to find models and meshes, not just in this package,
        # but in other ROS packages as well.
        # See: https://gazebosim.org/docs/garden/ros_integration#finding-resources
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            # Add the 'install' directory of your workspace to the resource path
            value=install_dir
        ),

        # Launch Gazebo Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': '-r empty.sdf',  # Use an empty world with a ground plane
                'on_exit_shutdown': 'True'
            }.items(),
        ),

        # Spawn the robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', robot_description_content,
                '-name', 'hexapod',
                '-z', '0.3'  # Spawn the robot 0.3 meters above the ground
            ],
            output='screen'
        ),
    ])