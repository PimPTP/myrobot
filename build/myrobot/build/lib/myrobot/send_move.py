#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from feetech_tuna.feetech_tuna import FeetechTuna

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

        self.declare_parameter('rw_id', 0)
        self.declare_parameter('rw_addr', 0)
        self.declare_parameter('rw_value', 0)
 
        self.create_service(Trigger, 'listservos', self.list_servos)
        self.create_service(Trigger, 'listregs', self.list_regs)
        self.create_service(Trigger, 'readreg', self.read_reg)
        self.create_service(Trigger, 'writereg', self.write_reg)

    def cmd_callback(self, msg: JointTrajectory):
        for pt in msg.points:
            for i, name in enumerate(msg.joint_names):
                if name not in JOINTS:
                    continue
                idx = JOINTS.index(name)
                sid = JOINT_ID[idx]

                pos_rad = pt.positions[i]

                vel_rpm0 = (pt.velocities[i] * 60.0 / (2 * math.pi)
                            if pt.velocities and i < len(pt.velocities) else 0.0)
                vel_rpm = vel_rpm0 or 50

                t_ms = pt.time_from_start.sec * 1000 + pt.time_from_start.nanosec / 1e6
                if t_ms == 0:
                    time_s  = abs(pos_rad) / (vel_rpm * 2 * math.pi / 60)
                    time_ms = max(int(time_s * 1000), 100)
                else:
                    time_ms = int(t_ms)
                    if vel_rpm0 == 0:
                        vel_rpm = abs(pos_rad) * 60 / (time_ms/1000 * 2 * math.pi)

                pos_cnt = int(pos_rad * (750.0 / math.pi))
                vel_cnt = int(vel_rpm / (240.0 / 1000.0))

                self.tuna.writeReg(sid, 42, pos_cnt)
                self.tuna.writeReg(sid, 44, time_ms)
                self.tuna.writeReg(sid, 46, vel_cnt)

    def publish_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        for name, sid in zip(JOINTS, JOINT_ID):
            pos_cnt = self.tuna.readReg(sid, 56)
            speed_cnt = self.tuna.readReg(sid, 58) or 0
            if pos_cnt is None:
                continue
            js.name.append(name)
            js.position.append(pos_cnt / (750.0 / math.pi))
            js.velocity.append(speed_cnt * (240.0/1000.0) * (2*math.pi/60))

        if js.name:
            self.pub_state.publish(js)


    def list_servos(self, req, res):
        servos = self.tuna.listServos()
        if servos:
            res.success = True
            res.message = ','.join(f"{s['id']}" for s in servos)
        else:
            res.success = False
            res.message = 'No servos found'
        return res

    def list_regs(self, req, res):
        sid = self.get_parameter('rw_id').value
        if sid == 0:
            res.success = False
            res.message = '0'
            return res
        regs = self.tuna.listRegs(sid)
        if not regs:
            res.success = False
            res.message = f'Servo {sid} not responding'
            return res
        res.success = True
        res.message = '; '.join(f"{r['addr']}:{r['name']}={r['value']}" for r in regs)
        return res

    def read_reg(self, req, res):
        sid = self.get_parameter('rw_id').value
        addr = self.get_parameter('rw_addr').value
        value = self.tuna.readReg(sid, addr)
        res.success = value is not None
        res.message = str(value if value is not None else '0')
        return res

    def write_reg(self, req, res):
        sid = self.get_parameter('rw_id').value
        addr = self.get_parameter('rw_addr').value
        value  = self.get_parameter('rw_value').value
        ok = self.tuna.writeReg(sid, addr, value)
        res.success = bool(ok)
        res.message = 'OK' if ok else '0'
        return res
    
    def destroy_node(self):
        self.tuna.closeSerialPort()
        super().destroy_node()

def main():
    rclpy.init()
    node = SendMove()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
