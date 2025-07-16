#!/usr/bin/env python3
import sys, rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton,
    QLineEdit, QGridLayout, QTextEdit
)
from PyQt6.QtCore import QTimer
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_ui')
        

class MonitorUI(QWidget):
    def __init__(self, node):
        super().__init__()
        

def main():
    rclpy.init()
    node = MonitorNode()

    app = QApplication(sys.argv)
    ui = MonitorUI(node)
    ui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(10)

    sys.exit(app.exec())

if __name__ == '__main__':
    main()
