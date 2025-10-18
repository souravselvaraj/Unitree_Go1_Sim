#!/usr/bin/env python3
# Move the torso in a vertical circle while the robot stands still,
# and show a live 3D matplotlib plot with a trail.
#
# Topics used (namespace robot1):
#   /robot1/robot_mode      : quadropted_msgs/msg/RobotModeCommand
#   /robot1/robot_velocity  : quadropted_msgs/msg/RobotVelocity (with geometry_msgs/Twist inside)
#
# Notes:
# - Matplotlib MUST run in the main thread. ROS2 spins in a background thread.
# - If the window doesn’t show, make sure you have a GUI backend installed (PyQt5 or similar).

import math
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from quadropted_msgs.msg import RobotModeCommand, RobotVelocity

# ---------- User-tunable params ----------
ROBOT_NS = "/robot1"        # change if you use another namespace
ROBOT_ID = 1                # matches your msg definition
RADIUS_M = 0.50             # 50 cm radius
CENTER_Z = 0.35             # base torso height (m), adjust for your model
FREQ_HZ = 2.0               # circle frequency in Hz (you said "2")
PUBLISH_RATE_HZ = 50        # how fast we publish velocity
TRAIL_SECONDS = 10          # trail length window in seconds in the plot
# ----------------------------------------

# Thread-safe buffer for the plotter
from queue import Queue
plot_queue = Queue()

class TorsoCircleNode(Node):
    def __init__(self):
        super().__init__('torso_circle_node')

        # Publishers
        self.mode_pub = self.create_publisher(RobotModeCommand, f'{ROBOT_NS}/robot_mode', 1)
        self.vel_pub  = self.create_publisher(RobotVelocity,    f'{ROBOT_NS}/robot_velocity', 10)

        # Send STAND once at start, then keep it alive every second
        self._send_stand()
        self.keep_stand_timer = self.create_timer(1.0, self._send_stand)

        # Control loop
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

        self.get_logger().info("✅ Torso vertical circle + live plot (window) is running")

    def _send_stand(self):
        msg = RobotModeCommand()
        msg.mode = "STAND"
        msg.robot_id = ROBOT_ID
        self.mode_pub.publish(msg)

    def _tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0

        # Vertical circle in X-Z plane:
        #   x(t) = R cos(2π f t)
        #   z(t) = CENTER_Z + R sin(2π f t)
        # Their time derivatives are the commanded linear velocities.
        w = 2.0 * math.pi * FREQ_HZ
        x = RADIUS_M * math.cos(w * t)
        z = CENTER_Z + RADIUS_M * math.sin(w * t)

        vx = -RADIUS_M * w * math.sin(w * t)   # dx/dt
        vz =  RADIUS_M * w * math.cos(w * t)   # dz/dt

        # Publish RobotVelocity
        vel = RobotVelocity()
        vel.robot_id = ROBOT_ID
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = 0.0
        twist.linear.z = vz
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        vel.cmd_vel = twist
        self.vel_pub.publish(vel)

        # Send a sample to the plotter (x,y,z). y stays 0 for a vertical circle.
        try:
            plot_queue.put_nowait((t, x, 0.0, z))
        except Exception:
            pass


# ---------------- Matplotlib live plot (main thread) ----------------
# IMPORTANT: import pyplot AFTER potential ROS init on some systems to avoid backend conflicts.
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (needed to register 3D)
from matplotlib.animation import FuncAnimation

class LivePlot3D:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Keep last N seconds of trail
        max_points = max(5, int(TRAIL_SECONDS * PUBLISH_RATE_HZ))
        self.buf_t  = deque(maxlen=max_points)
        self.buf_x  = deque(maxlen=max_points)
        self.buf_y  = deque(maxlen=max_points)
        self.buf_z  = deque(maxlen=max_points)

        # Pre-create artists
        (self.trail_line,) = self.ax.plot([], [], [], lw=2)
        self.point = self.ax.scatter([], [], [], s=30)

        # Axes limits & labels
        pad = 0.02
        x_lim = (-RADIUS_M - pad, RADIUS_M + pad)
        y_lim = (-0.05, 0.05)
        z_lim = (CENTER_Z - RADIUS_M - pad, CENTER_Z + RADIUS_M + pad)
        self.ax.set_xlim(x_lim)
        self.ax.set_ylim(y_lim)
        self.ax.set_zlim(z_lim)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('Torso Vertical Circle (X–Z)')

        # Keep a reference to the animation
        self.ani = FuncAnimation(self.fig, self._update, interval=100, blit=False, cache_frame_data=False)

    def _update(self, _):
        # Drain queue fast (only keep the newest)
        drained = 0
        while not plot_queue.empty():
            t, x, y, z = plot_queue.get()
            self.buf_t.append(t)
            self.buf_x.append(x)
            self.buf_y.append(y)
            self.buf_z.append(z)
            drained += 1

        if drained == 0:
            return self.trail_line, self.point

        # Update artists
        self.trail_line.set_data(self.buf_x, self.buf_y)
        self.trail_line.set_3d_properties(self.buf_z)

        # scatter needs to be recreated to move a single point cleanly
        self.point.remove()
        self.point = self.ax.scatter(self.buf_x[-1], self.buf_y[-1], self.buf_z[-1], s=40)

        return self.trail_line, self.point

    def show(self):
        plt.show()


def ros_spin_in_background(node: Node):
    # Spin rclpy in a dedicated thread so the main thread can run the GUI
    def _spin():
        rclpy.spin(node)

    th = threading.Thread(target=_spin, daemon=True)
    th.start()
    return th


def main():
    rclpy.init()

    node = TorsoCircleNode()
    spin_thread = ros_spin_in_background(node)

    try:
        # Run the GUI in the main thread
        plotter = LivePlot3D()
        plotter.show()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.get_logger().info("Shutting down…")
        node.destroy_node()
        rclpy.shutdown()
        # Give the spin thread a moment to finish
        time.sleep(0.2)


if __name__ == "__main__":
    main()
