#!/home/pim0ubuntu/myrobot_ws/.venv/bin/python
import rclpy, math, threading, time, signal
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from . import Right_Elbow_Angle_v2 as Right_Elbow
from . import param

class CamInput(Node):
    def __init__(self):
        super().__init__('cam_input')
        qos_profile = QoSProfile(depth=10)

        self.pub_cmd = self.create_publisher(JointTrajectory, '/joint_commands', qos_profile)
        self.pub_cam = self.create_publisher(Float64, '/cam_angle', qos_profile)
        self.sub_cam = self.create_subscription(Float64, '/cam_angle', self.cam_callback, qos_profile)

        self.mp_thread = threading.Thread(target=Right_Elbow.RightElbow, daemon=True)
        self.mp_thread.start()

        self.pos_inlast = None
        self.init_move()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        pos_in = Right_Elbow.R_ANGLE
        self.pub_cam.publish(Float64(data=float(pos_in)))

    def cam_callback(self, msg):
        pos_in = msg.data
        if pos_in < 0.0 or pos_in > 180.0:
            return
        if self.pos_inlast is not None and abs(pos_in-self.pos_inlast) < 10.0:
            return
        self.pos_inlast = pos_in
        pos_rad = -0.5 + (pos_in / 180.0) * math.pi

        traj = JointTrajectory()
        traj.joint_names = ['joint_4']
        pt = JointTrajectoryPoint()
        pt.positions = [pos_rad]
        pt.velocities = [10.0]
        traj.points.append(pt)
        self.pub_cmd.publish(traj)

    def init_move(self):
        traj = JointTrajectory()
        traj.joint_names = param.JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [3.14, 0.9, -1.5, -0.5, 0.0, 0.0]
        pt.velocities = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
        traj.points.append(pt)
        self.pub_cmd.publish(traj)

    def default_move(self):
        traj = JointTrajectory()
        traj.joint_names = param.JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pt.velocities = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
        traj.points.append(pt)
        self.pub_cmd.publish(traj)

def main():
    rclpy.init()
    node = CamInput()

    def sigint_handler(signum, frame):
        node.default_move()
        time.sleep(0.1)
        rclpy.shutdown()
    signal.signal(signal.SIGINT, sigint_handler)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()