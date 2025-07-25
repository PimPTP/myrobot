from math import sin, cos
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        qos_profile = QoSProfile(depth=10)

        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.joint_name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.name_to_index = {name: i for i, name in enumerate(self.joint_name)}
        self.create_subscription(JointState, 'cmd_joint', self.joint_callback, qos_profile)

        self.timer = self.create_timer(0.1, self.update)

    def joint_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            idx = self.name_to_index.get(name)
            if idx is not None:
                self.joint_positions[idx] = pos

    def update(self):
        now = self.get_clock().now()

        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = self.joint_name
        joint_state.position = self.joint_positions


        odom_trans = TransformStamped()
        odom_trans.header.stamp = now.to_msg()
        odom_trans.header.frame_id = 'world'
        odom_trans.child_frame_id = 'base_link'
        odom_trans.transform.translation.x = 0.0
        odom_trans.transform.translation.y = 0.0
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = euler_to_quaternion(0, 0, 0)

        self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(odom_trans)

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    rclpy.init()
    node =StatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()