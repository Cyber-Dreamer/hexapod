import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pybullet_ros',
            executable='pybullet_ros_node',
            name='pybullet_sim',
            output='screen',
            parameters=[
                {'robot_description': os.path.join(
                    os.getenv('HOME'),
                    'Documents/hexapod/hexapod/urdf/hexapod.urdf'
                )},
                {'use_gui': True}
            ]
        )
    ])