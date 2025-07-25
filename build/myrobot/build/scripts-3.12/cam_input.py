#!/usr/bin/python3
import rclpy, threading
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from . import Right_Elbow_Angle_v2 as Right_Elbow

class CamInput(Node):
    def __init__(self):
        super().__init__('cam_input')
        qos_profile = QoSProfile(depth=10)

        self.pub_cmd = self.create_publisher(JointTrajectory, '/joint_commands', qos_profile)
        self.pub_cam = self.create_publisher(Float64, '/cam_angle', qos_profile)
        self.sub_cam = self.create_subscription(Float64, '/cam_angle', self.cam_callback, qos_profile)

        self.mp_thread = threading.Thread(target=Right_Elbow.RightElbow, daemon=True)
        self.mp_thread.start()

        self.init_move()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def init_move(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        pt = JointTrajectoryPoint()
        pt.positions = [3.14, 0.9, -1.5, -0.5]
        pt.velocities = [10.0, 10.0, 10.0, 10.0]
        traj.points.append(pt)
        self.pub_cmd.publish(traj)

    def timer_callback(self):
        pos_in = Right_Elbow.R_ANGLE
        self.pub_cam.publish(Float64(data=float(pos_in)))

    def cam_callback(self, msg):
        pos_in = msg.data
        if pos_in <= 0.0 or pos_in > 180.0:
            return
        pos_rad = -0.5 + (pos_in / 180.0) * 1.7
        pos_rad = max(-0.5, min(1.2, pos_rad))

        traj = JointTrajectory()
        traj.joint_names = ['joint_4']
        pt = JointTrajectoryPoint()
        pt.positions = [pos_rad]
        pt.velocities = [10.0]
        traj.points.append(pt)
        self.pub_cmd.publish(traj)

def main():
    rclpy.init()
    node = CamInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()