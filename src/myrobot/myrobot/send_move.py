#!/usr/bin/env python3
import math, rclpy, time, signal, threading
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from feetech_tuna.feetech_tuna import FeetechTuna
from .servo_srv import ServoSrv
from . import param

class SendMove(Node):
    def __init__(self):
        super().__init__('send_move')
        qos_profile = QoSProfile(depth=10)

        self.tuna = FeetechTuna()
        if not self.tuna.openSerialPort(param.SERIAL_PORT, param.BAUD_RATE):
            self.get_logger().error(f"Failed to open serial port {param.SERIAL_PORT}")
            raise RuntimeError("Cannot open serial port")

        self.sub_cmd = self.create_subscription(JointTrajectory, '/joint_commands', self.cmd_callback, qos_profile)
        self.pub_state = self.create_publisher(JointState, '/joint_states', qos_profile)
        self.pub_monitor = self.create_publisher(String, '/joint_monitor', qos_profile)

        self.default_move()
        self.servo_srv = ServoSrv(self, self.tuna)

        self.timer = self.create_timer(0.1, self.publish_states)

    def cmd_callback(self, msg: JointTrajectory):
        threading.Thread(target=self.send_trajectory, args=(msg,), daemon=True).start()

    def send_trajectory(self, msg: JointTrajectory):
        if not msg.points:
            return

        for pt in msg.points:
            for i, name in enumerate(msg.joint_names):
                if name not in param.JOINTS:
                    continue
                idx = param.JOINTS.index(name)
                sid = param.JOINT_ID[idx]
                limit = param.JOINT_LIMITS[idx]

                pos_rad = pt.positions[i]
                pos_rad = max(limit[0], min(limit[1], pos_rad))
                pos_cnt = int(round((pos_rad + math.pi) * (4096.0 / (2 * math.pi))))
                pos_cnt = max(0, min(4095, pos_cnt))

                if pt.velocities and i < len(pt.velocities):
                    vel_rpm = pt.velocities[i]
                else:
                    vel_rpm = 10
                vel_cnt  = int(vel_rpm *4096.0 / 60)
                vel_cnt  = max(-8000, min(8000, vel_cnt))

                self.tuna.writeReg(sid, 46, vel_cnt)
                self.tuna.writeReg(sid, 42, pos_cnt)

    def publish_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        lines = []

        for name, sid in zip(param.JOINTS, param.JOINT_ID):
            try:
                pos_cnt = self.tuna.readReg(sid, 56)
                vel_cnt = self.tuna.readReg(sid, 58)
                voltage = self.tuna.readReg(sid, 62)
                current = self.tuna.readReg(sid, 69)
                temp = self.tuna.readReg(sid, 63)
                moving_flag = self.tuna.readReg(sid, 66)

                if None in (pos_cnt, vel_cnt, voltage, current, temp, moving_flag):
                    continue

                pos_rad = (pos_cnt / 4096.0) * 2 * math.pi - math.pi
                vel_rpm = vel_cnt * 60 / 4096.0
                voltage = voltage / 10.0
                moving_str = "Moving" if moving_flag else "Fixed"

                js.name.append(name)
                js.position.append(pos_rad)
                js.velocity.append(vel_rpm)

                lines.append(
                    f"{name}  Position: {pos_rad:6.3f}rad  Velocity: {vel_rpm:6.2f}rpm  "
                    f"Voltage : {voltage}V  Current: {current}mA  Temp: {temp}°C  {moving_str}")

            except Exception:
                pass

        if js.name:
            self.pub_state.publish(js)
            self.pub_monitor.publish(String(data='\n'.join(lines)))

    def default_move(self):
        pos_rad = 0.0
        vel_cnt = int(10 *4096 / 60)
        for idx, sid in enumerate(param.JOINT_ID):
            pos_cnt = int(round((pos_rad + math.pi) * (4096.0 / (2 * math.pi))))
            pos_cnt = max(0, min(4095, pos_cnt))
            self.tuna.writeReg(sid, 46, vel_cnt)
            self.tuna.writeReg(sid, 42, pos_cnt)

    def wait_stopped(self, timeout_sec=5.0):
        start = time.time()
        while time.time() - start < timeout_sec:
            all_stopped = True
            for sid in param.JOINT_ID:
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