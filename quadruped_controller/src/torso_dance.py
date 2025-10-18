#!/usr/bin/env python3
# Extreme 3D torso dance with IMU stabilization + RViz trail
# Works with Ignition Gazebo setup from your workspace
# Author: you + ChatGPT

import math
import time
from collections import deque
import sys
sys.path.append('/home/sourav/ros2_ws/src/quadruped_controller/src')


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, ColorRGBA
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker

from quadropted_msgs.msg import RobotModeCommand
from quadruped_controller.InverseKinematics import robot_IK
from quadruped_controller.RobotController import RobotController

RATE_HZ = 100.0

# ==== EXTREME AMPLITUDES (but still with safety clamps) ====
AMP_X = 0.03     # 3 cm forward/back
AMP_Y = 0.03     # 3 cm side/side
AMP_Z = 0.06     # 6 cm vertical bounce
AMP_PITCH = math.radians(10.0)  # 10 deg
AMP_ROLL  = math.radians(10.0)  # 10 deg
AMP_YAW   = math.radians(15.0)  # 15 deg

# Additional gentle modulation for "dance" vibe
W = 1.2         # base angular frequency (rad/s)
W_YAW = 0.7     # yaw slower for style

# IMU stabilization (PD) to fight tilt drift while dancing
Kp_roll  = 0.35
Kd_roll  = 0.02
Kp_pitch = 0.35
Kd_pitch = 0.02

# Safety clamps (absolute)
MAX_XY = 0.05
MAX_Z = 0.08
MAX_RP = math.radians(12.0)
MAX_YAW = math.radians(20.0)

class Torso3DDanceExtreme(Node):
    def __init__(self):
        super().__init__('torso_3d_dance_extreme')

        # Geometry as in your controller
        body = [0.3762, 0.0935]
        legs = [0.0, 0.08, 0.213, 0.213]

        # Robot wrapper (same as your controller uses)
        self.robot = RobotController.Robot(self, body, legs, use_imu=False, robot_id=1)
        self.ik = robot_IK.InverseKinematics(body, legs)

        # Publishers
        self.joint_pub = self.create_publisher(Float64MultiArray, '/robot1/joint_group_controller/commands', 10)
        self.mode_pub  = self.create_publisher(RobotModeCommand, '/robot1/robot_mode', 10)

        # IMU
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.prev_roll_err = 0.0
        self.prev_pitch_err = 0.0
        self.prev_imu_t = None
        self.create_subscription(Imu, '/robot1/imu_plugin/out', self.imu_cb, 10)

        # RViz marker for live 3D trail of torso COM
        self.trail_pub = self.create_publisher(Marker, '/robot1/torso_trail', 10)
        self.trail = deque(maxlen=500)  # sliding window
        self.marker = self._make_trail_marker()

        # Put robot into STAND once
        self._send_stand_mode()

        # Dance loop
        self.t0 = time.time()
        self.timer = self.create_timer(1.0 / RATE_HZ, self.step)

        self.get_logger().info('🔥 Extreme 3D torso dance + IMU stabilization started! Open RViz and add Marker on /robot1/torso_trail')

    # ----------------- helpers -----------------
    def _send_stand_mode(self):
        msg = RobotModeCommand()
        msg.mode = 'STAND'
        msg.robot_id = 1
        for _ in range(5):
            self.mode_pub.publish(msg)
        self.get_logger().info('✅ STAND mode requested')

    @staticmethod
    def quat_to_euler(qx, qy, qz, qw):
        # ZYX convention (yaw, pitch, roll)
        # roll (x-axis), pitch (y-axis), yaw (z-axis)
        sinr_cosp = 2.0 * (qw*qx + qy*qz)
        cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw*qy - qz*qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi/2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (qw*qz + qx*qy)
        cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def imu_cb(self, msg: Imu):
        r, p, _ = self.quat_to_euler(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        )
        self.imu_roll = r
        self.imu_pitch = p

    def _make_trail_marker(self) -> Marker:
        m = Marker()
        m.header.frame_id = 'robot1/base'   # shows torso trajectory in body frame; change to 'map' for world
        m.ns = 'torso_trail'
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.01  # line width
        m.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=0.9)
        m.pose.orientation.w = 1.0
        return m

    def publish_trail(self, x, y, z):
        from geometry_msgs.msg import Point
        pt = Point(x=float(x), y=float(y), z=float(z))
        self.trail.append(pt)
        self.marker.points = list(self.trail)
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.trail_pub.publish(self.marker)

    def clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    # ----------------- main loop -----------------
    def step(self):
        t = time.time() - self.t0

        # Base dance signals
        dx = AMP_X * math.sin(W * t)
        dy = AMP_Y * math.sin(W * t + math.pi/2.0)
        dz = AMP_Z * (0.6 + 0.4*math.sin(2.0*W * t))   # keep positive bounce

        roll_des  = AMP_ROLL  * math.sin(W * t + math.pi/3.0)
        pitch_des = AMP_PITCH * math.sin(W * t)
        yaw_des   = AMP_YAW   * math.sin(W_YAW * t)

        # IMU stabilization (PD) reduces measured roll/pitch drift
        roll_err  = -self.imu_roll
        pitch_err = -self.imu_pitch

        # crude dt using node clock
        now = self.get_clock().now().seconds_nanoseconds()
        if self.prev_imu_t is None:
            dt = 1.0 / RATE_HZ
        else:
            dt = max(1e-3, (now[0] + now[1]*1e-9) - self.prev_imu_t)
        self.prev_imu_t = (now[0] + now[1]*1e-9)

        d_roll  = (roll_err  - self.prev_roll_err)  / dt
        d_pitch = (pitch_err - self.prev_pitch_err) / dt
        self.prev_roll_err  = roll_err
        self.prev_pitch_err = pitch_err

        roll_cmd  = roll_des  + (Kp_roll  * roll_err)  + (Kd_roll  * d_roll)
        pitch_cmd = pitch_des + (Kp_pitch * pitch_err) + (Kd_pitch * d_pitch)

        # Safety clamps
        dx = self.clamp(dx, -MAX_XY, MAX_XY)
        dy = self.clamp(dy, -MAX_XY, MAX_XY)
        dz = self.clamp(dz, 0.0, MAX_Z)  # keep >=0
        roll_cmd  = self.clamp(roll_cmd,  -MAX_RP,  MAX_RP)
        pitch_cmd = self.clamp(pitch_cmd, -MAX_RP,  MAX_RP)
        yaw_cmd   = self.clamp(yaw_des,   -MAX_YAW, MAX_YAW)

        # Get nominal foot targets from your controller (feet stay planted in STAND)
        leg_positions = self.robot.run()   # velocities are zero by default in STAND
        # Ask IK for joint angles to realize the body offset/orientation
        try:
            q = self.ik.inverse_kinematics(
                leg_positions,
                dx, dy, dz,
                roll_cmd, pitch_cmd, yaw_cmd
            )
            msg = Float64MultiArray()
            msg.data = [float(v) for v in q]
            self.joint_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'IK/command error: {e}')
            return

        # RViz trail (in torso/body frame for a nice relative “dance” path)
        self.publish_trail(dx, dy, dz)

def main():
    rclpy.init()
    node = Torso3DDanceExtreme()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
