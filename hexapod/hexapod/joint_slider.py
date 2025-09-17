#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import time

class JointSliderNode(Node):
    def __init__(self):
        super().__init__('hexapod_joint_slider')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_names = [
            # Replace with your actual joint names from the URDF
            'hip_1', 'ties_1', 'foot_1',
            'hip_2', 'ties_2', 'foot_2',
            'hip_3', 'ties_3', 'foot_3',
            'hip_4', 'ties_4', 'foot_4',
            'hip_5', 'ties_5', 'foot_5',
            'hip_6', 'ties_6', 'foot_6',
        ]
        self.positions = [0.0] * len(self.joint_names)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.get_logger().info('Hexapod joint slider node started.')
        self.print_help()
        self.run_cli()

    def print_help(self):
        print('\nCommands:')
        print('  set <joint_index> <position>   # Set joint at index to position (radians)')
        print('  show                           # Show current joint positions')
        print('  help                           # Show this help')
        print('  exit                           # Quit')
        print('\nJoint indices:')
        for i, name in enumerate(self.joint_names):
            print(f'  {i}: {name}')

    def run_cli(self):
        while rclpy.ok():
            try:
                cmd = input('> ').strip().split()
                if not cmd:
                    continue
                if cmd[0] == 'set' and len(cmd) == 3:
                    idx = int(cmd[1])
                    pos = float(cmd[2])
                    if 0 <= idx < len(self.positions):
                        self.positions[idx] = pos
                        print(f'Set {self.joint_names[idx]} to {pos} radians')
                    else:
                        print('Invalid joint index')
                elif cmd[0] == 'show':
                    for i, (name, pos) in enumerate(zip(self.joint_names, self.positions)):
                        print(f'{i}: {name} = {pos:.3f}')
                elif cmd[0] == 'help':
                    self.print_help()
                elif cmd[0] == 'exit':
                    print('Exiting...')
                    sys.exit(0)
                else:
                    print('Unknown command. Type "help".')
            except (KeyboardInterrupt, EOFError):
                print('\nExiting...')
                sys.exit(0)
            except Exception as e:
                print(f'Error: {e}')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointSliderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
