#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64
from px4_msgs.msg import VehicleOdometry


class CameraYawLookAtPatternCenter(Node):
    """
    Command camera yaw so it always points to the SAME center used by the scan pattern.

    Frames:
      - Gazebo SDF / your pattern generator: ENU in 2D (x=East, y=North)
      - PX4 VehicleOdometry: NED (x=North, y=East, z=Down)
      - Rover yaw from PX4 odom quaternion: yaw about +Down, with
          yaw=0 -> North
          yaw=+pi/2 -> East   (clockwise when viewed from above)

    We compute:
      target_world_yaw_ned = atan2(delta_east, delta_north)
      desired_camera_joint_yaw = wrap_to_pi(target_world_yaw_ned - rover_yaw + camera_yaw_offset_rad)

    Published cmd is a *relative* yaw angle (camera wrt rover body) to the Gazebo joint cmd topic.
    """

    def __init__(self):
        super().__init__('camera_yaw_look_at_pattern_center')

        # --- Parameters ---
        self.declare_parameter(
            'cmd_topic',
            '/world/rover_lunar/model/r1_rover_camera_0/joint/camera_yaw_joint/cmd_pos'
        )
        self.declare_parameter('odom_topic', '/fmu/out/vehicle_odometry')

        # Center used by test_pattern.py (ENU: x=East, y=North)
        # test_pattern.py sets: self.cylinder_center_enu = (5.0, 0.0)
        self.declare_parameter('center_east_enu', 5.0)
        self.declare_parameter('center_north_enu', 0.0)

        # Camera yaw offset (rad): use this if camera joint 0 rad is not rover-forward.
        self.declare_parameter('camera_yaw_offset_rad', 0.0)

        # Control / smoothing
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('max_yaw_rate_rad_s', 2.0)   # higher -> tracks better
        self.declare_parameter('yaw_deadband_rad', 0.02)    # higher -> less micro-jitter
        self.declare_parameter('cmd_lowpass_alpha', 0.0)    # 0 disables; try 0.1 if noisy
        self.declare_parameter('debug', True)

        # --- Read params ---
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)

        self.center_east_enu = float(self.get_parameter('center_east_enu').value)
        self.center_north_enu = float(self.get_parameter('center_north_enu').value)

        # Convert ENU (E,N) -> NED (N,E) exactly as in test_pattern.py
        self.center_north_ned, self.center_east_ned = self.enu_xy_to_ned_xy(
            self.center_east_enu, self.center_north_enu
        )

        self.cam_offset = float(self.get_parameter('camera_yaw_offset_rad').value)

        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate_rad_s').value)
        self.deadband = float(self.get_parameter('yaw_deadband_rad').value)
        self.alpha = float(self.get_parameter('cmd_lowpass_alpha').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.alpha = max(0.0, min(1.0, self.alpha))

        # --- QoS (PX4 odom typically best-effort) ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(Float64, self.cmd_topic, 10)
        self.sub = self.create_subscription(VehicleOdometry, self.odom_topic, self.on_odom, qos)

        # --- State ---
        self.have_odom = False
        self.north = 0.0
        self.east = 0.0
        self.rover_yaw = 0.0

        # Command state in [-pi, pi]
        self.cmd_wrapped = 0.0
        self.filtered_cmd_wrapped = 0.0
        self.initialized_cmd = False

        self.last_t = self.get_clock().now()
        self.debug_counter = 0

        period = 1.0 / max(self.rate_hz, 1e-3)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            "CameraYawLookAtPatternCenter started\n"
            f"  cmd_topic:   {self.cmd_topic}\n"
            f"  odom_topic:  {self.odom_topic}\n"
            f"  center ENU:  (E={self.center_east_enu:.3f}, N={self.center_north_enu:.3f})\n"
            f"  center NED:  (N={self.center_north_ned:.3f}, E={self.center_east_ned:.3f})\n"
            f"  cam_offset:  {self.cam_offset:.6f} rad\n"
            f"  rate:        {self.rate_hz:.1f} Hz\n"
            f"  max_rate:    {self.max_yaw_rate:.3f} rad/s\n"
            f"  deadband:    {self.deadband:.4f} rad\n"
            f"  lpf alpha:   {self.alpha:.3f}\n"
            f"  debug:       {self.debug}"
        )

    # --------------------
    # Frame helpers
    # --------------------
    @staticmethod
    def enu_xy_to_ned_xy(x_east, y_north):
        """
        ENU (x=East, y=North) -> PX4 local NED (x=North, y=East)
        NED.x = North = ENU.y
        NED.y = East  = ENU.x
        """
        return float(y_north), float(x_east)

    @staticmethod
    def wrap_to_pi(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def shortest_angular_error(target_wrapped: float, current_wrapped: float) -> float:
        return CameraYawLookAtPatternCenter.wrap_to_pi(target_wrapped - current_wrapped)

    @staticmethod
    def yaw_from_quaternion_wxyz(q):
        # q = [w, x, y, z]
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # --------------------
    # ROS callbacks
    # --------------------
    def on_odom(self, msg: VehicleOdometry):
        self.north = float(msg.position[0])
        self.east = float(msg.position[1])
        self.rover_yaw = self.yaw_from_quaternion_wxyz(msg.q)
        self.have_odom = True

    def on_timer(self):
        if not self.have_odom:
            return

        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        dt = max(dt, 1e-3)
        self.last_t = now

        # Vector rover -> center in NED
        delta_n = self.center_north_ned - self.north
        delta_e = self.center_east_ned - self.east

        # Bearing in NED: yaw = atan2(East, North)
        target_world_yaw = math.atan2(delta_e, delta_n)

        # Desired camera yaw RELATIVE to rover body
        desired_wrapped = self.wrap_to_pi(target_world_yaw - self.rover_yaw + self.cam_offset)

        if self.debug and (self.debug_counter % 30 == 0):
            self.get_logger().info(
                f"Rover NED: (N={self.north:.3f}, E={self.east:.3f}), "
                f"rover_yaw={math.degrees(self.rover_yaw):.1f}° | "
                f"Target world yaw={math.degrees(target_world_yaw):.1f}° | "
                f"Desired cam yaw={math.degrees(desired_wrapped):.1f}° | "
                f"Cmd={math.degrees(self.cmd_wrapped):.1f}°"
            )
        self.debug_counter += 1

        # Initialize command to avoid a large transient
        if not self.initialized_cmd:
            self.cmd_wrapped = desired_wrapped
            self.filtered_cmd_wrapped = desired_wrapped
            self.initialized_cmd = True

        # Shortest error
        err = self.shortest_angular_error(desired_wrapped, self.cmd_wrapped)

        # Deadband to prevent micro-hunting
        if abs(err) <= self.deadband:
            cmd_next = self.cmd_wrapped
        else:
            # Rate limit
            if self.max_yaw_rate > 0.0:
                max_step = self.max_yaw_rate * dt
                step = max(-max_step, min(max_step, err))
                cmd_next = self.wrap_to_pi(self.cmd_wrapped + step)
            else:
                cmd_next = desired_wrapped

        self.cmd_wrapped = cmd_next

        # Optional low-pass filter in wrapped domain
        if self.alpha > 0.0:
            ferr = self.shortest_angular_error(self.cmd_wrapped, self.filtered_cmd_wrapped)
            self.filtered_cmd_wrapped = self.wrap_to_pi(self.filtered_cmd_wrapped + self.alpha * ferr)
            cmd_pub = self.filtered_cmd_wrapped
        else:
            cmd_pub = self.cmd_wrapped

        out = Float64()
        out.data = float(cmd_pub)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CameraYawLookAtPatternCenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down camera_yaw_look_at_pattern_center")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
