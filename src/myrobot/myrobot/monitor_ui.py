#!/usr/bin/env python3
import sys, rclpy, re
from rclpy.node import Node
from rclpy.qos import QoSProfile
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton,
    QLineEdit, QGridLayout, QTextEdit, QGroupBox)
from PyQt6.QtCore import QTimer
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from . import param

PARAMS = ["Position (rad)", "Velocity (rpm)", "Voltage (V)", "Current (mA)", "Temp (°C)", "Status"]

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_ui')
        qos_profile = QoSProfile(depth=10)

        self.sub_monitor = self.create_subscription(String, '/joint_monitor', self.monitor_callback, qos_profile)
        self.pub_cmd = self.create_publisher(JointTrajectory, '/joint_commands', qos_profile)
        
        self.latest_monitor = ''

    def monitor_callback(self, msg):
        self.latest_monitor = msg.data

    def send_command(self, positions, velocities):
        msg = JointTrajectory()
        msg.joint_names = param.JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.velocities = [float(v) for v in velocities]
        msg.points.append(point)
        self.pub_cmd.publish(msg)

class MonitorUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Joint Monitor UI")

        layout = QVBoxLayout()

        monitor_box = QGroupBox("Joint Monitor")
        monitor_layout = QVBoxLayout()

        self.monitor_grid = QGridLayout()
        self.monitor_labels = {key: [] for key in PARAMS}

        for j, joint in enumerate(param.JOINTS):
            self.monitor_grid.addWidget(QLabel(f"<b>{joint}</b>"), 0, j + 1)

        for i, params in enumerate(PARAMS):
            self.monitor_grid.addWidget(QLabel(params), i + 1, 0)
            for j in range(len(param.JOINTS)):
                label = QLabel("-")
                self.monitor_grid.addWidget(label, i + 1, j + 1)
                self.monitor_labels[params].append(label)

        monitor_layout.addLayout(self.monitor_grid)
        monitor_box.setLayout(monitor_layout)
        layout.addWidget(monitor_box)

        command_box = QGroupBox("Joint Command Input")
        grid = QGridLayout()
        self.pos_inputs = []
        self.vel_inputs = []

        for j, joint in enumerate(param.JOINTS):
            grid.addWidget(QLabel(f"<b>{joint}</b>"), 0, j + 1)

        grid.addWidget(QLabel("Position (rad)"), 1, 0)
        for j in range(len(param.JOINTS)):
            pos_input = QLineEdit("0.0")
            self.pos_inputs.append(pos_input)
            grid.addWidget(pos_input, 1, j + 1)

        grid.addWidget(QLabel("Velocity (rpm)"), 2, 0)
        for j in range(len(param.JOINTS)):
            vel_input = QLineEdit("10.0")
            self.vel_inputs.append(vel_input)
            grid.addWidget(vel_input, 2, j + 1)

        self.send_button = QPushButton("Send Command")
        self.send_button.clicked.connect(self.on_send_command)
        grid.addWidget(self.send_button, 3, 0, 1, len(param.JOINTS) + 1)

        command_box.setLayout(grid)
        layout.addWidget(command_box)

        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_monitor)
        self.timer.start(100)
        
    def update_monitor(self):
        data_lines = self.node.latest_monitor.strip().splitlines()
        for j, line in enumerate(data_lines):
            match = re.search(
                r"joint_\d+\s+Position:\s*([-\d.]+)rad\s+Velocity:\s*([-\d.]+)rpm\s+Voltage\s*:\s*([\d.]+)V\s+Current:\s*(\d+)mA\s+Temp:\s*(\d+)°C\s+(\w+)", line)
            if not match:
                continue
            pos, vel, volt, curr, temp, status = match.groups()
            
            if j < len(param.JOINTS):
                self.monitor_labels["Position (rad)"][j].setText(pos)
                self.monitor_labels["Velocity (rpm)"][j].setText(vel)
                self.monitor_labels["Voltage (V)"][j].setText(volt)
                self.monitor_labels["Current (mA)"][j].setText(curr)
                self.monitor_labels["Temp (°C)"][j].setText(temp)
                self.monitor_labels["Status"][j].setText(status)

    def on_send_command(self):
        positions = [inp.text() for inp in self.pos_inputs]
        velocities = [inp.text() for inp in self.vel_inputs]
        self.node.send_command(positions, velocities)
        

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
