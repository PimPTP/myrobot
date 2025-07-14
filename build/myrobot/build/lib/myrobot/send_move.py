#!/usr/bin/env python3
import math, rclpy, time, signal
from rclpy.node import Node
from rclpy.qos import QoSProfile
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from feetech_tuna.feetech_tuna import FeetechTuna
from .servo_srv import ServoSrv

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

JOINTS = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
JOINT_ID = [10, 11, 12, 13, 14, 15]

class SendMove(Node):
    def __init__(self):
        super().__init__('send_move')
        qos_profile = QoSProfile(depth=10)

        self.tuna = FeetechTuna()
        if not self.tuna.openSerialPort(SERIAL_PORT, BAUD_RATE):
            self.get_logger().error(f"Failed to open serial port {SERIAL_PORT}")
            raise RuntimeError("Cannot open serial port")

        self.sub_cmd = self.create_subscription(
            JointTrajectory,
            '/joint_commands',
            self.cmd_callback,
            qos_profile)

        self.pub_state = self.create_publisher(
            JointState,
            '/joint_states',
            qos_profile)

        self.timer = self.create_timer(0.1, self.publish_states)

        self.default_move()
        self.servo_srv = ServoSrv(self, self.tuna)

    def cmd_callback(self, msg: JointTrajectory):
        if not msg.points:
            return

        for pt in msg.points:
            for i, name in enumerate(msg.joint_names):
                if name not in JOINTS:
                    continue
                idx = JOINTS.index(name)
                sid = JOINT_ID[idx]

                pos_rad = pt.positions[i]
                pos_cnt = int(round(((pos_rad + math.pi) * (4096.0 / (2*math.pi))))) % 4096  
                pos_cnt  = max(0, min(4095, pos_cnt))

                if pt.velocities and i < len(pt.velocities):
                    vel_rpm = pt.velocities[i] 
                else:
                    vel_rpm = 100
                vel_cnt  = int(vel_rpm / 0.24)           
                vel_cnt  = max(1, min(1000, vel_cnt)) 

                t_ms = pt.time_from_start.sec * 1000 + pt.time_from_start.nanosec / 1e6
                if t_ms == 0:
                    time_ms = max(int(abs(pos_rad) / vel_rpm * 1000), 100)
                else:
                    time_ms = int(t_ms)
                    if vel_rpm == 0:
                        vel_rpm = abs(pos_rad) / (time_ms / 1000)
                        vel_cnt = max(1, min(1000, int(vel_rpm / 0.24)))
                time_ms += idx * 1000
                time_ms = max(1, min(30000, time_ms)) 

                self.tuna.writeReg(sid, 46, vel_cnt)
                self.tuna.writeReg(sid, 44, time_ms)
                self.tuna.writeReg(sid, 42, pos_cnt)

    def publish_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        for name, sid in zip(JOINTS, JOINT_ID):
            pos_cnt = self.tuna.readReg(sid, 56)
            speed_cnt = self.tuna.readReg(sid, 58)
            if pos_cnt is None:
                continue
            js.name.append(name)
            js.position.append((pos_cnt / 4096.0) * 2*math.pi - math.pi)
            js.velocity.append(speed_cnt * 0.24)

        if js.name:
            self.pub_state.publish(js)

    def default_move(self):
        pos_rad = 0.0
        vel_cnt = int(100/0.24)
        time_ms = 1000
        for idx, sid in enumerate(JOINT_ID):
            pos_cnt = int(round(((pos_rad + math.pi) * (4096.0 / (2*math.pi))))) % 4096
            self.tuna.writeReg(sid, 46, vel_cnt)
            self.tuna.writeReg(sid, 44, time_ms)
            self.tuna.writeReg(sid, 42, pos_cnt)

    def wait_stopped(self, timeout_sec=5.0):
        start = time.time()
        while time.time() - start < timeout_sec:
            all_stopped = True
            for sid in JOINT_ID:
                try:
                    moving_flag = self.tuna.readReg(sid, 66)
                    if moving_flag is None or moving_flag != 0:
                        all_stopped = False
                        break
                except Exception:
                    all_stopped = False
                    break
            if all_stopped:
                return True
            time.sleep(0.05)
        return False
    
    def destroy_node(self):
        self.timer.cancel()
        super().destroy_node()

def main():
    rclpy.init()
    node = SendMove()

    shutdown_flag = False
    def sigint_handler(signum, frame):
        nonlocal shutdown_flag
        shutdown_flag = True
        rclpy.shutdown()
    signal.signal(signal.SIGINT, sigint_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if shutdown_flag:
            try:
                node.default_move()
                node.wait_stopped(timeout_sec=5.0)
            except Exception:
                pass
        try:
            node.tuna.closeSerialPort()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()