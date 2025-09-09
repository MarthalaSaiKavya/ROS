#!/usr/bin/env python3
import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute
from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

def ang_wrap(a):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))

class NDrawer(Node):
    def __init__(self):
        super().__init__('p1d2_sai_marthala')
        
        try:
            pkg_share = get_package_share_directory('p1d2_sai_marthala')
            self.logo_path = os.path.join(pkg_share, 'resource', 'ncsu_logo.png')
            if os.path.exists(self.logo_path):
                self.get_logger().info(f"Loaded NCSU logo from: {self.logo_path}")
            else:
                self.get_logger().warn("⚠️ NCSU logo not found — overlay will skip background.")
        except Exception as e:
            self.get_logger().warn(f"Could not find package. Skipping logo. Error: {e}")
            self.logo_path = ""

        # Parameters
        self.declare_parameter('kp_linear', 2.0)
        self.declare_parameter('kp_angular', 8.0) # Increased for sharper corners
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('angle_threshold', 0.08)
        self.declare_parameter('goal_tolerance', 0.05)

        # ROS pubs/subs/services
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.setpen_cli = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        # --- FINAL WAYPOINTS (GEOMETRY CORRECTED) ---
        # These points have been fine-tuned to make the diagonal lines parallel
        # and the vertical/horizontal lines perfectly straight.
        self.start = (3.8, 6.0)
        self.waypoints = [
            (3.8, 8.5),   # 1. Left bar up
            (4.3, 8.5),   # 2. Top-left cap
            (6.8, 6.4),   # 3. Inner diagonal end (SHAPE CORRECTION)
            (6.8, 8.5),   # 4. Right bar inner up
            (7.3, 8.5),   # 5. Top-right cap
            (7.3, 6.0),   # 6. Right bar down
            (6.8, 6.0),   # 7. Bottom-right cap
            (4.3, 8.1),   # 8. Outer diagonal start (SHAPE CORRECTION)
            (4.3, 6.0),   # 9. Left bar inner down
            (3.8, 6.0),   # 10. Close shape
        ]
        # --- END OF CORRECTION ---

        # State variables
        self.pose = None
        self.idx = -1
        self.started = False
        self.done = False
        self.path = []
        self.timer = self.create_timer(0.05, self.control_step)
        self.get_logger().info("✅ NDrawer node started — waiting for pose and services...")

    def pose_cb(self, msg: Pose):
        self.pose = msg
        if not self.done and self.started:
            self.path.append((msg.x, msg.y))

    def control_step(self):
        if self.done or self.pose is None:
            return

        if not self.started:
            if not self.setpen_cli.service_is_ready() or not self.teleport_cli.service_is_ready():
                self.get_logger().info("Waiting for turtlesim services...")
                return
            
            pen_up = SetPen.Request(r=0, g=0, b=0, width=3, off=1)
            self.setpen_cli.call_async(pen_up)
            
            tp = TeleportAbsolute.Request(x=self.start[0], y=self.start[1], theta=math.pi / 2)
            self.teleport_cli.call_async(tp)
            
            pen_down = SetPen.Request(r=0, g=0, b=255, width=3, off=0)
            self.setpen_cli.call_async(pen_down)

            self.idx = 0
            self.started = True
            self.get_logger().info(f"✏️ Teleported to start {self.start}. Starting to draw outline...")
            return

        if self.idx >= len(self.waypoints):
            self.pub.publish(Twist())
            self.done = True
            self.get_logger().info("✅ Finished drawing N outline.")
            self.plot_overlay()
            rclpy.shutdown()
            return

        gx, gy = self.waypoints[self.idx]
        dx, dy = gx - self.pose.x, gy - self.pose.y
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_err = ang_wrap(desired_yaw - self.pose.theta)
        
        kp_lin = self.get_parameter('kp_linear').value
        kp_ang = self.get_parameter('kp_angular').value
        vmax = self.get_parameter('max_speed').value
        a_thr = self.get_parameter('angle_threshold').value
        tol = self.get_parameter('goal_tolerance').value

        if dist < tol:
            self.get_logger().info(f"Waypoint {self.idx} reached.")
            self.pub.publish(Twist())
            self.idx += 1
            return

        cmd = Twist()
        if abs(yaw_err) < a_thr:
            cmd.linear.x = min(vmax, kp_lin * dist)
        else:
            cmd.linear.x = 0.0
        cmd.angular.z = max(-8.0, min(8.0, kp_ang * yaw_err))
        self.pub.publish(cmd)

    def plot_overlay(self):
        if not self.path:
            self.get_logger().warn("No path data recorded, skipping plot.")
            return
        
        try:
            plt.figure()
            if self.logo_path and os.path.exists(self.logo_path):
                img = mpimg.imread(self.logo_path)
                plt.imshow(img, extent=[0, 11, 0, 11])

            xs, ys = zip(*self.path)
            plt.plot(xs, ys, color='blue', linewidth=3)

            plt.xlim([0, 11])
            plt.ylim([0, 11])
            plt.title("Turtlesim N Outline vs. NCSU Logo")
            plt.gca().set_aspect('equal', adjustable='box')
            plt.grid(True)
            plt.show()
        except Exception as e:
            self.get_logger().error(f"Failed to generate plot: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

