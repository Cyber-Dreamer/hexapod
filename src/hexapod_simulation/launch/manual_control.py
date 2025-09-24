from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Joint State Publisher GUI node
    # This node provides a GUI to manually control the joints of your robot
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        joint_state_publisher_gui_node
    ])
