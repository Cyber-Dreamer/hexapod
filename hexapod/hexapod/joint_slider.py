#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import sys
try:
    from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QLabel, QVBoxLayout, QHBoxLayout
    from PyQt5.QtCore import Qt
    pyqt_available = True
except ImportError:
    pyqt_available = False

class JointSliderNode(Node):
    def __init__(self):
        super().__init__('hexapod_joint_slider')
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )
        self.joint_names = [
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

    def slider_changed(self, idx, val, label):
        pos = val / 100.0  # Map slider to -3.14..3.14
        self.positions[idx] = pos
        label.setText(f"{self.joint_names[idx]}: {pos:.2f} rad")

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
        msg = Float64MultiArray()
        msg.data = self.positions
        self.publisher_.publish(msg)

def main():
    import threading
    rclpy.init()
    node = JointSliderNode()

    if pyqt_available:
        def ros_spin():
            rclpy.spin(node)
        ros_thread = threading.Thread(target=ros_spin, daemon=True)
        ros_thread.start()

        # Start the Qt GUI in the main thread
        app = QApplication(sys.argv)
        window = QWidget()
        window.setWindowTitle('Hexapod Joint Sliders')
        layout = QVBoxLayout()
        node.slider_labels = []
        node.sliders = []
        for i, name in enumerate(node.joint_names):
            hbox = QHBoxLayout()
            label = QLabel(f"{name}: 0.00 rad")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-314)  # -3.14 rad
            slider.setMaximum(314)   # 3.14 rad
            slider.setValue(0)
            slider.setSingleStep(1)
            slider.valueChanged.connect(lambda val, idx=i, l=label: node.slider_changed(idx, val, l))
            hbox.addWidget(label)
            hbox.addWidget(slider)
            layout.addLayout(hbox)
            node.slider_labels.append(label)
            node.sliders.append(slider)
        window.setLayout(layout)
        window.show()
        node._qt_app = app
        node._qt_window = window
        sys.exit(app.exec_())
    else:
        node.print_help()
        node.run_cli()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
