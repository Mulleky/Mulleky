#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    VehicleOdometry,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)


class State(Enum):
    INIT = 0
    WAIT_FOR_ODOM = 1
    START_OFFBOARD = 2
    RUN_PATTERN = 3
    HOLD_FACE_CENTER = 4
    FINISHED = 5


class ScanPatternNode(Node):
    def __init__(self):
        super().__init__('scan_pattern_node')

        # --- Parameters ---
        self.declare_parameter('num_circles', 3)
        self.declare_parameter('inner_radius', 5.0)
        self.declare_parameter('radius_increment', 4.0)
        self.declare_parameter('points_per_circle', 24)   # denser than before for better overlap

        self.declare_parameter('velocity', 1.0)
        self.declare_parameter('position_tolerance', 1.5)
        self.declare_parameter('hold_duration_s', 2.0)
        self.declare_parameter('lookahead_m', 0.6)

        num_circles = int(self.get_parameter('num_circles').value)
        inner_radius = float(self.get_parameter('inner_radius').value)
        radius_increment = float(self.get_parameter('radius_increment').value)
        points_per_circle = int(self.get_parameter('points_per_circle').value)

        self.velocity = float(self.get_parameter('velocity').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.hold_duration_s = float(self.get_parameter('hold_duration_s').value)
        self.lookahead_m = float(self.get_parameter('lookahead_m').value)

        # Cylinder center from world SDF in ENU: (x=East, y=North)
        self.cylinder_center_enu = (5.0, 0.0)

        # Convert cylinder center to PX4 NED (x=North, y=East)
        self.cylinder_center_ned = self.enu_xy_to_ned_xy(*self.cylinder_center_enu)

        # --- NEW PATTERN: full 360° concentric circles (multi-ring orbits) ---
        # We generate in ENU, then convert each waypoint to NED for PX4.
        self.waypoints_ned = []
        self.waypoints_enu_dbg = []

        for circle_idx in range(num_circles):
            radius = inner_radius + (circle_idx * radius_increment)
            for i in range(points_per_circle):
                angle = (2.0 * math.pi * i) / float(points_per_circle)
                ex = self.cylinder_center_enu[0] + radius * math.cos(angle)  # East
                ny = self.cylinder_center_enu[1] + radius * math.sin(angle)  # North

                self.waypoints_enu_dbg.append((ex, ny))
                x_ned, y_ned = self.enu_xy_to_ned_xy(ex, ny)
                self.waypoints_ned.append((x_ned, y_ned))

        # Return to the FIRST waypoint (departure = return point)
        if len(self.waypoints_ned) > 0:
            self.waypoints_ned.append(self.waypoints_ned[0])
            self.waypoints_enu_dbg.append(self.waypoints_enu_dbg[0])

        self.current_wp_idx = 0

        # PX4 state
        self.state = State.INIT
        self.odom_received = False
        self.current_position = [0.0, 0.0, 0.0]  # NED: [North, East, Down]
        self.current_yaw = 0.0
        self.offboard_set = False
        self.armed = False

        # HOLD state bookkeeping
        self.hold_until_us = 0
        self.hold_wp_x = 0.0
        self.hold_wp_y = 0.0
        self.hold_wp_z = 0.0
        self.hold_wp_yaw = 0.0

        # Segment start for along-track acceptance
        self.segment_start_xy = None

        # --- Publishers ---
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # --- QoS for odometry subscription (match PX4: BEST_EFFORT) ---
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Subscribers ---
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_best_effort
        )

        # --- Timers ---
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"Generated {len(self.waypoints_ned)} waypoints in {num_circles} full 360° concentric circles "
            f"(points_per_circle={points_per_circle})"
        )
        self.get_logger().info(f"Cylinder center ENU: {self.cylinder_center_enu}  -> NED: {self.cylinder_center_ned}")
        self.get_logger().info("ScanPatternNode initialized")
        self.state = State.WAIT_FOR_ODOM

    # --------------------
    # Frame helpers
    # --------------------

    @staticmethod
    def enu_xy_to_ned_xy(x_east, y_north):
        """
        Convert ENU (x=East, y=North) to PX4 local NED (x=North, y=East).
        NED.x = North = ENU.y
        NED.y = East  = ENU.x
        """
        return float(y_north), float(x_east)

    @staticmethod
    def ned_xy_to_enu_xy(x_north, y_east):
        """
        Convert NED (x=North, y=East) to ENU (x=East, y=North).
        ENU.x = East  = NED.y
        ENU.y = North = NED.x
        """
        return float(y_east), float(x_north)

    # --------------------
    # Callbacks & helpers
    # --------------------

    def odom_callback(self, msg: VehicleOdometry):
        # VehicleOdometry position is PX4 local frame (typically NED):
        # position[0]=North, position[1]=East, position[2]=Down
        self.current_position = [msg.position[0], msg.position[1], msg.position[2]]
        self.current_yaw = self.yaw_from_quaternion(msg.q)
        self.odom_received = True

    @staticmethod
    def yaw_from_quaternion(q):
        # q = [w, x, y, z]
        w, x, y, z = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_pub.publish(msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.get_logger().info("Sending ARM command")
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )
        self.armed = True

    def disarm(self):
        self.get_logger().info("Sending DISARM command")
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )
        self.armed = False

    def set_offboard_mode(self):
        self.get_logger().info("Setting OFFBOARD mode")
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )
        self.offboard_set = True

    def distance_to_waypoint(self, x, y):
        dx = x - self.current_position[0]
        dy = y - self.current_position[1]
        return math.sqrt(dx * dx + dy * dy)

    def yaw_to_face_center_ned(self, x, y):
        """
        Compute yaw in PX4 NED frame to face the cylinder center.
        yaw = atan2(East_delta, North_delta)
        """
        cx, cy = self.cylinder_center_ned
        dx_n = cx - x
        dy_e = cy - y
        return math.atan2(dy_e, dx_n)

    def lookat_micro_target_xy(self, x, y, step_m):
        """
        Move a tiny step from (x,y) toward cylinder center in NED.
        """
        cx, cy = self.cylinder_center_ned
        dx = cx - x
        dy = cy - y
        norm = math.sqrt(dx * dx + dy * dy)
        if norm < 1e-6:
            return x, y
        ux = dx / norm
        uy = dy / norm
        return x + step_m * ux, y + step_m * uy

    def ensure_segment_start(self):
        if self.segment_start_xy is None:
            if self.current_wp_idx == 0:
                self.segment_start_xy = (self.current_position[0], self.current_position[1])
            else:
                self.segment_start_xy = self.waypoints_ned[self.current_wp_idx - 1]

    def reached_by_alongtrack_or_radius(self, wp_x, wp_y):
        dist = self.distance_to_waypoint(wp_x, wp_y)
        if dist < self.position_tolerance:
            return True, dist, 0.0

        self.ensure_segment_start()
        sx, sy = self.segment_start_xy

        seg_x = wp_x - sx
        seg_y = wp_y - sy
        seg_len = math.sqrt(seg_x * seg_x + seg_y * seg_y)
        if seg_len < 1e-6:
            return False, dist, 0.0

        ux = seg_x / seg_len
        uy = seg_y / seg_len

        rx = self.current_position[0] - sx
        ry = self.current_position[1] - sy
        along = rx * ux + ry * uy

        if along >= (seg_len - self.position_tolerance):
            return True, dist, along

        return False, dist, along

    def start_hold_at_waypoint(self, wp_x, wp_y, wp_z):
        self.hold_wp_x = float(wp_x)
        self.hold_wp_y = float(wp_y)
        self.hold_wp_z = float(wp_z)
        self.hold_wp_yaw = float(self.yaw_to_face_center_ned(wp_x, wp_y))

        now_us = self.get_clock().now().nanoseconds // 1000
        self.hold_until_us = now_us + int(self.hold_duration_s * 1_000_000)

        self.state = State.HOLD_FACE_CENTER

        enu_wp = self.ned_xy_to_enu_xy(wp_x, wp_y)
        self.get_logger().info(
            f"Holding at waypoint {self.current_wp_idx} for {self.hold_duration_s:.1f}s "
            f"while facing cylinder (yaw={self.hold_wp_yaw:.2f} rad). "
            f"WP ENU=({enu_wp[0]:.2f},{enu_wp[1]:.2f})"
        )

    def finish_pattern(self, wp_x, wp_y, wp_z, wp_yaw):
        self.get_logger().info("Final waypoint reached (returned to start). Holding position and disarming rover.")
        self.publish_trajectory_setpoint(wp_x, wp_y, wp_z, wp_yaw)
        if self.armed:
            self.disarm()
        self.state = State.FINISHED

    # --------------
    # Main loop
    # --------------

    def control_loop(self):
        if self.state == State.WAIT_FOR_ODOM:
            if self.odom_received:
                self.get_logger().info("Odometry received, starting OFFBOARD pipeline")
                self.state = State.START_OFFBOARD
            return

        # Keep OFFBOARD messages streaming
        if self.state in (State.START_OFFBOARD, State.RUN_PATTERN, State.HOLD_FACE_CENTER):
            self.publish_offboard_control_mode()

        if self.state == State.START_OFFBOARD:
            # Hold current position
            x0, y0, z0 = self.current_position
            self.publish_trajectory_setpoint(x0, y0, z0, self.current_yaw)

            if not self.armed:
                self.arm()

            if not self.offboard_set:
                self.set_offboard_mode()

            if self.armed and self.offboard_set:
                self.get_logger().info("OFFBOARD + ARM requested, starting scan pattern")
                self.segment_start_xy = (self.current_position[0], self.current_position[1])
                self.state = State.RUN_PATTERN
            return

        if self.state == State.RUN_PATTERN:
            wp_x, wp_y = self.waypoints_ned[self.current_wp_idx]
            wp_z = float(self.current_position[2])  # keep current Down

            # Maintain current yaw during transit; yaw correction is enforced during HOLD
            self.publish_trajectory_setpoint(wp_x, wp_y, wp_z, self.current_yaw)

            arrived, dist, along = self.reached_by_alongtrack_or_radius(wp_x, wp_y)
            if arrived:
                self.get_logger().info(
                    f"Reached waypoint {self.current_wp_idx} "
                    f"NED target=({wp_x:.2f},{wp_y:.2f}) "
                    f"NED pos=({self.current_position[0]:.2f},{self.current_position[1]:.2f}) "
                    f"dist={dist:.2f} along={along:.2f}"
                )
                self.start_hold_at_waypoint(wp_x, wp_y, wp_z)
            return

        if self.state == State.HOLD_FACE_CENTER:
            current_z = float(self.current_position[2])

            # Micro-target toward cylinder to force steering/heading alignment (in NED)
            tx, ty = self.lookat_micro_target_xy(self.hold_wp_x, self.hold_wp_y, self.lookahead_m)

            self.publish_trajectory_setpoint(tx, ty, current_z, self.hold_wp_yaw)

            now_us = self.get_clock().now().nanoseconds // 1000
            if now_us >= self.hold_until_us:
                if self.current_wp_idx == len(self.waypoints_ned) - 1:
                    self.finish_pattern(self.hold_wp_x, self.hold_wp_y, current_z, self.hold_wp_yaw)
                else:
                    self.current_wp_idx += 1
                    self.segment_start_xy = self.waypoints_ned[self.current_wp_idx - 1]
                    self.state = State.RUN_PATTERN
            return

        if self.state == State.FINISHED:
            return


def main(args=None):
    rclpy.init(args=args)
    node = ScanPatternNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down ScanPatternNode')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
