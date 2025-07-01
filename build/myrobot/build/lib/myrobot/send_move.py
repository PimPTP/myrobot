#!/usr/bin/env python3
import math, struct, serial, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
HEADER = b'\x55\x55'

JOINTS = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
SID = {name: 10 + i for i, name in enumerate(JOINTS)}

TICK_MAX = 4095
RAD_PER_TICK = 2 * math.pi / TICK_MAX
RPM_PER_TICK= 0.11
MS_PER_TIME_TICK= 11.2

def rad_to_tick(rad: float) -> int:
    return int((rad % (2*math.pi)) / RAD_PER_TICK)

def tick_to_rad(tick: int) -> float:
    return (tick % (TICK_MAX+1)) * RAD_PER_TICK

def rads_to_tick(rads: float) -> int:
    rpm = rads * 60 / (2*math.pi)
    spd = min(int(abs(rpm) / RPM_PER_TICK), 1023)
    if rpm < 0:
        spd |= 0x400
    return spd

def tick_to_rads(tick: int) -> float:
    direction = -1 if (tick & 0x400) else 1
    spd = tick & 0x3FF
    rpm = spd * RPM_PER_TICK * direction
    return rpm * 2*math.pi / 60

class SendMove(Node):
    def __init__(self):
        super().__init__('send_move')
        qos_profile = QoSProfile(depth=10)

        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

        self.create_subscription(JointTrajectory, '/joint_commands', self.cmd_callback, qos_profile)
        self.state_pub = self.create_publisher(JointState, '/joint_states', qos_profile)

        self.timer = self.create_timer(0.1, self.publish_states)
    
    def cmd_callback(self, msg: JointTrajectory):
        if not msg.points: return
        point = msg.points[-1]
        duration = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        duration = max(duration, 0.05)
        time_tick = min(int(math.ceil(duration*1000/MS_PER_TIME_TICK)), 65535)

        for idx, name in enumerate(msg.joint_names):
            if name not in SID or idx >= len(point.positions): continue
            pos_tick = rad_to_tick(point.positions[idx])
            vel_rads = point.velocities[idx] if idx < len(point.velocities) else 0.0
            speed_tick = rads_to_tick(vel_rads)

            self.send_command(SID[name], pos_tick, speed_tick, time_tick)

    def send_command(self, sid: int, pos_tick: int, speed_tick: int, time_tick: int):
        payload = b'\x2A' + struct.pack('<HHH', pos_tick, speed_tick, time_tick)
        self.write_packet(sid, 0x03, payload)

    def write_packet(self, sid: int, inst: int, params: bytes):
        packet = bytearray(HEADER + bytes([sid, len(params)+3, inst]) + params)
        packet.append((~sum(packet[2:]) & 0xFF))
        self.ser.write(packet)
    
    def publish_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        for name in JOINTS:
            pos, spd = self.read_pos(SID[name])
            if pos is None:
                continue
            js.name.append(name)
            js.position.append(tick_to_rad(pos))
            js.velocity.append(tick_to_rads(spd))

        if js.name:
            self.state_pub.publish(js)

    def read_pos(self, sid: int):
        self.write_packet(sid, 0x02, b'\x38\x04')
        if self.ser.read(2) != HEADER:
            return None, None
        hdr = self.ser.read(3)
        if len(hdr) < 3:
            return None, None
        _, length, err = hdr
        data = self.ser.read(length-2)
        if err or len(data) != length-2:
            return None, None
        pos_tick = (data[0] | (data[1] << 8)) & TICK_MAX
        spd_tick = (data[2] | (data[3] << 8)) & 0x7FF
        return pos_tick, spd_tick

def main():
    rclpy.init()
    rclpy.spin(SendMove())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
