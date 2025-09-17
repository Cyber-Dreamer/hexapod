from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim'],
            output='screen'
        ),
        # Spawn the ground plane
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-name', 'ground_plane', '-string',
                '<sdf version="1.6">\n'
                '  <model name="ground_plane">\n'
                '    <static>true</static>\n'
                '    <link name="link">\n'
                '      <collision name="collision">\n'
                '        <geometry>\n'
                '          <plane>\n'
                '            <normal>0 0 1</normal>\n'
                '            <size>100 100</size>\n'
                '          </plane>\n'
                '        </geometry>\n'
                '      </collision>\n'
                '      <visual name="visual">\n'
                '        <geometry>\n'
                '          <plane>\n'
                '            <normal>0 0 1</normal>\n'
                '            <size>100 100</size>\n'
                '          </plane>\n'
                '        </geometry>\n'
                '        <material>\n'
                '          <script>\n'
                '            <uri>file://media/materials/scripts/gazebo.material</uri>\n'
                '            <name>Gazebo/Grey</name>\n'
                '          </script>\n'
                '        </material>\n'
                '      </visual>\n'
                '    </link>\n'
                '  </model>\n'
                '</sdf>'
            ]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-name', 'hexapod', '-file', urdf_path]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),
    ])
