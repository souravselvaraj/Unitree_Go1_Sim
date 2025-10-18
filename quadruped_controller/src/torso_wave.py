#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Torso Figure-8 tilt while standing + live Matplotlib plot (no threading)

- Publishes joint positions to: /robot1/joint_group_controller/commands
- Sets mode to STAND on:       /robot1/robot_mode   (quadropted_msgs/RobotModeCommand)
- Subscribes to odom on:       /robot1/odom         (nav_msgs/Odometry) for plotting
- Uses 3-DOF leg IK (hip abduction, hip flexion, knee)
"""

import math
import time
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

# custom msgs in your workspace
from quadropted_msgs.msg import RobotModeCommand
from InverseKinematics import robot_IK
from RobotController import RobotController  # only for geometry constants

# Matplotlib in the main thread
import matplotlib
matplotlib.use("Qt5Agg")  # if you’re on a headless server, switch to "Agg"
import matplotlib.pyplot as plt


def quat_to_euler_xyz(x, y, z, w):
    """Convert quaternion to roll (X), pitch (Y), yaw (Z) in radians.
    No tf_transformations dependency."""
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class TorsoFigure8Stand(Node):
    def __init__(self):
        super().__init__('torso_figure8_stand')

        # ---------- Parameters ----------
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('roll_amp_deg', 8.0)   # amplitude of roll   (deg)
        self.declare_parameter('pitch_amp_deg', 8.0)  # amplitude of pitch  (deg)
        self.declare_parameter('freq_hz', 0.35)       # base frequency (Hz)
        self.declare_parameter('stance_height', 0.27) # meters
        self.declare_parameter('stance_x', 0.20)      # hip-to-foot nominal x (m)
        self.declare_parameter('stance_y', 0.11)      # hip-to-foot nominal y (m)
        self.declare_parameter('verbose', False)

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.roll_amp = math.radians(float(self.get_parameter('roll_amp_deg').value))
        self.pitch_amp = math.radians(float(self.get_parameter('pitch_amp_deg').value))
        self.freq_hz = float(self.get_parameter('freq_hz').value)
        self.stance_height = float(self.get_parameter('stance_height').value)
        self.stance_x = float(self.get_parameter('stance_x').value)
        self.stance_y = float(self.get_parameter('stance_y').value)
        self.verbose = bool(self.get_parameter('verbose').value)

        if self.verbose:
            self.get_logger().info(
                f"Params: roll_amp={math.degrees(self.roll_amp):.1f}deg, "
                f"pitch_amp={math.degrees(self.pitch_amp):.1f}deg, "
                f"freq={self.freq_hz}Hz, stance=({self.stance_x},{self.stance_y},{self.stance_height})"
            )

        # ---------- Robot geometry & IK ----------
        # This matches your earlier controller (length/half-width etc.)
        body = [0.3762, 0.0935]                      # [length, half_width]
        legs = [0.0, 0.08, 0.213, 0.213]             # [hip_offset, link1, link2, ...] (from your repo)
        self.ik = robot_IK.InverseKinematics(body, legs)

        # Nominal foot targets in body frame (x, y, z). 4x3
        # Order: FL, FR, RL, RR
        z = -abs(self.stance_height)
        self.nominal_feet = np.array([
            [ +self.stance_x, +self.stance_y, z],   # FL
            [ +self.stance_x, -self.stance_y, z],   # FR
            [ -self.stance_x, +self.stance_y, z],   # RL
            [ -self.stance_x, -self.stance_y, z],   # RR
        ], dtype=float)

        # ---------- Publishers / Subscribers ----------
        self.pub_joints = self.create_publisher(
            Float64MultiArray, '/robot1/joint_group_controller/commands', 10)

        self.pub_mode = self.create_publisher(
            RobotModeCommand, '/robot1/robot_mode', 10)

        self.sub_odom = self.create_subscription(
            Odometry, '/robot1/odom', self._odom_cb, 10)

        # Set STAND mode once at startup (a few times to be safe)
        self._send_stand_mode(repeat=10)

        # ---------- Timing ----------
        self.rate_hz = 60.0
        self.dt = 1.0 / self.rate_hz
        self.t0 = time.time()
        self.timer = self.create_timer(self.dt, self._tick)

        # ---------- Plot setup (main thread, no extra threads) ----------
        plt.ion()
        self.fig, self.ax = plt.subplots(1, 1, figsize=(7, 4))
        self.ax.set_title("Torso Figure-8 Tilt (roll & pitch)")
        self.ax.set_xlabel("time (s)")
        self.ax.set_ylabel("angle (deg)")
        self.ax.grid(True, linestyle='--', alpha=0.35)

        self.t_hist: List[float] = []
        self.roll_cmd_hist: List[float] = []
        self.pitch_cmd_hist: List[float] = []
        self.roll_meas_hist: List[float] = []
        self.pitch_meas_hist: List[float] = []

        (self.line_roll_cmd,) = self.ax.plot([], [], label='roll_cmd')
        (self.line_pitch_cmd,) = self.ax.plot([], [], label='pitch_cmd')
        (self.line_roll_meas,) = self.ax.plot([], [], label='roll_meas')
        (self.line_pitch_meas,) = self.ax.plot([], [], label='pitch_meas')
        self.ax.legend(loc='upper right')

        self.last_meas_rpy = (0.0, 0.0, 0.0)  # (roll, pitch, yaw)

        self.get_logger().info("✅ Torso Figure-8 + Live Plot started (STAND mode).")

    # ---------------------- Callbacks ----------------------
    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.last_meas_rpy = quat_to_euler_xyz(q.x, q.y, q.z, q.w)

    # ---------------------- Helpers ------------------------
    def _send_stand_mode(self, repeat=3):
        msg = RobotModeCommand()
        msg.mode = "STAND"
        msg.robot_id = self.robot_id
        for _ in range(repeat):
            self.pub_mode.publish(msg)

    def _figure8(self, t):
        """Lissajous-style figure-8 in tilt space."""
        w = 2.0 * math.pi * self.freq_hz
        roll = self.roll_amp * math.sin(w * t)
        pitch = self.pitch_amp * math.sin(2.0 * w * t)  # double frequency for the “8”
        return roll, pitch

    @staticmethod
    def _enforce_3_per_leg_per_foot(angles_per_leg: List[np.ndarray]) -> np.ndarray:
        """Force [3] per leg; concatenate to [12]."""
        clean = []
        for leg in angles_per_leg:
            a = np.array(leg).reshape(-1)
            if a.size >= 3:
                a = a[:3]
            else:
                a = np.pad(a, (0, 3 - a.size), mode='constant', constant_values=0.0)
            clean.append(a)
        return np.concatenate(clean)

    # ---------------------- Main Loop ----------------------
    def _tick(self):
        t = time.time() - self.t0

        # Desired torso orientation (roll & pitch). We keep body position & yaw = 0.
        roll, pitch = self._figure8(t)
        yaw = 0.0
        dx = dy = 0.0
        dz = 0.0  # keep the same height (feet fixed at nominal z)

        # Inverse kinematics for each leg; your IK takes (feet, dx,dy,dz, r,p,y)
        try:
            # IMPORTANT: stop any drift — use fixed nominal feet in body frame.
            feet_targets = np.array(self.nominal_feet, dtype=float)

            # IK returns per-leg joint angles; make sure we publish exactly 12 values (3 per leg)
            # If your IK returns flat 12 already, this still safely enforces [12].
            ik_out = self.ik.inverse_kinematics(feet_targets, dx, dy, dz, roll, pitch, yaw)

            # Support both shapes: (4,3) or (12,)
            if isinstance(ik_out, (list, tuple)):
                ik_out = np.array(ik_out)

            if ik_out.ndim == 1:  # assume [12]
                # split into 4 legs, enforce 3 each, concat back
                legs_split = np.split(ik_out, 4)
                joint_cmd = self._enforce_3_per_leg_per_foot(legs_split)
            elif ik_out.ndim == 2:  # assume (4,3) or (4,>=3)
                legs_split = [ik_out[i, :] for i in range(4)]
                joint_cmd = self._enforce_3_per_leg_per_foot(legs_split)
            else:
                raise ValueError(f"Unexpected IK output shape: {ik_out.shape}")

            if joint_cmd.size != 12:
                raise ValueError(f"Joint vector must be 12, got {joint_cmd.size}")

            msg = Float64MultiArray()
            msg.data = joint_cmd.tolist()
            self.pub_joints.publish(msg)

        except Exception as e:
            self.get_logger().error(f"IK/Publish error: {e}")
            return

        # ----- Plot (commanded vs measured) -----
        self.t_hist.append(t)
        self.roll_cmd_hist.append(math.degrees(roll))
        self.pitch_cmd_hist.append(math.degrees(pitch))
        r_meas, p_meas, _ = self.last_meas_rpy
        self.roll_meas_hist.append(math.degrees(r_meas))
        self.pitch_meas_hist.append(math.degrees(p_meas))

        # limit history to keep UI fast
        max_pts = 1200
        if len(self.t_hist) > max_pts:
            self.t_hist = self.t_hist[-max_pts:]
            self.roll_cmd_hist = self.roll_cmd_hist[-max_pts:]
            self.pitch_cmd_hist = self.pitch_cmd_hist[-max_pts:]
            self.roll_meas_hist = self.roll_meas_hist[-max_pts:]
            self.pitch_meas_hist = self.pitch_meas_hist[-max_pts:]

        # update lines
        self.line_roll_cmd.set_data(self.t_hist, self.roll_cmd_hist)
        self.line_pitch_cmd.set_data(self.t_hist, self.pitch_cmd_hist)
        self.line_roll_meas.set_data(self.t_hist, self.roll_meas_hist)
        self.line_pitch_meas.set_data(self.t_hist, self.pitch_meas_hist)

        # autoscale x to last N seconds, y to ±(amp*1.5)
        window_sec = 20.0
        tmax = self.t_hist[-1]
        tmin = max(0.0, tmax - window_sec)
        self.ax.set_xlim(tmin, tmax)

        yspan = max(5.0,
                    1.5 * max(1e-3 + max(map(abs, self.roll_cmd_hist[-300:] or [0])),
                              1e-3 + max(map(abs, self.pitch_cmd_hist[-300:] or [0])),
                              1e-3 + max(map(abs, self.roll_meas_hist[-300:] or [0])),
                              1e-3 + max(map(abs, self.pitch_meas_hist[-300:] or [0]))))
        self.ax.set_ylim(-yspan, yspan)

        self.ax.figure.canvas.draw()
        plt.pause(0.001)  # safe inside timer in main thread


def main():
    rclpy.init()
    node = TorsoFigure8Stand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
