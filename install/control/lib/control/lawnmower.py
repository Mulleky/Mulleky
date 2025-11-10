#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
import numpy as np
import time


class LawnMowerOffboard(Node):
    def __init__(self):
        super().__init__('lawnmower_offboard_horizontal_pattern')

        # QoS (per PX4 reference)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # PX4 interface
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)

        # Pattern bounds and parameters
        self.min_x = 2.5
        self.max_x = 22.5
        self.min_y = 2.5
        self.max_y = 22.5
        self.row_spacing = 3.0  # horizontal lane spacing
        self.z = 0.0

        # Start at upper-left corner
        self.start_x = self.min_x
        self.start_y = self.max_y

        # Mission settings
        self.rate_hz = 10.0
        self.timer_period = 1.0 / self.rate_hz
        self.waypoints = self._generate_horizontal_lawnmower_pattern()
        self.home_wp = (self.start_x, self.start_y, self.z, 0.0)
        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints.')

        # State
        self.current_wp_idx = 0
        self.hold_cycles = int(1.0 * self.rate_hz)
        self.hold_counter = 0
        self.counter = 0
        self.position_received = False
        self.armed = False
        self.phase = 'mission'
        self.current_position = np.zeros(3)

        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info('Node started — publishing at %.1f Hz' % self.rate_hz)

    # ---------------------------------------------------
    # Generate continuous horizontal lawnmower pattern
    # ---------------------------------------------------
    def _generate_horizontal_lawnmower_pattern(self):
        """
        Generates connected (x, y, z, yaw) waypoints for horizontal lanes.
        Starts at top-left corner (min_x, max_y), moves east across top row,
        steps downward by row_spacing, then moves west, repeating until bottom.
        """
        wps = []

        x_min = self.min_x
        x_max = self.max_x
        y_min = self.min_y
        y_max = self.max_y
        dy = self.row_spacing

        y = y_max
        going_east = True

        # Start position
        wps.append((x_min, y_max, self.z, 0.0))

        while y >= y_min:
            if going_east:
                # go east across the row
                wps.append((x_max, y, self.z, 0.0))
            else:
                # go west across the row
                wps.append((x_min, y, self.z, 0.0))

            # move down to next row
            y_next = y - dy
            if y_next >= y_min:
                # vertical step to next row start
                if going_east:
                    wps.append((x_max, y_next, self.z, 0.0))
                else:
                    wps.append((x_min, y_next, self.z, 0.0))

            y = y_next
            going_east = not going_east

        # Return to starting point at the end
        wps.append((x_min, y_max, self.z, 0.0))
        return wps

    # ---------------------------------------------------
    # PX4 communication helpers
    # ---------------------------------------------------
    def odom_cb(self, msg: VehicleOdometry):
        self.current_position = np.array(msg.position)
        if not self.position_received:
            self.get_logger().info(f'Got odometry: x={msg.position[0]:.2f}, y={msg.position[1]:.2f}')
            self.position_received = True

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x, y, z, yaw=0.0):
        sp = TrajectorySetpoint()
        sp.position = [float(x), float(y), float(z)]
        sp.velocity = [float('nan')] * 3
        sp.yaw = float(yaw)
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(sp)

    def send_vehicle_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = int(command)
        msg.param1, msg.param2 = float(p1), float(p2)
        msg.target_system = msg.source_system = 1
        msg.target_component = msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def engage_offboard(self):
        self.get_logger().info('Requesting OFFBOARD mode')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def arm(self):
        self.get_logger().info('Sending ARM')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.armed = True

    def disarm(self):
        self.get_logger().info('Sending DISARM')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.armed = False

    # ---------------------------------------------------
    # Main control loop
    # ---------------------------------------------------
    def control_loop(self):
        self.publish_offboard_mode()

        if not self.position_received:
            self.publish_setpoint(self.start_x, self.start_y, self.z)
            return

        if not self.armed:
            self.publish_setpoint(self.start_x, self.start_y, self.z)
            self.counter += 1
            if self.counter >= self.rate_hz:
                self.engage_offboard()
                time.sleep(0.05)
                self.arm()
            return

        if self.phase == 'mission':
            if self.current_wp_idx < len(self.waypoints):
                x, y, z, yaw = self.waypoints[self.current_wp_idx]
                self.publish_setpoint(x, y, z, yaw)
                self.hold_counter += 1

                if self.hold_counter == 1:
                    self.get_logger().info(f'WP {self.current_wp_idx+1}/{len(self.waypoints)} → x:{x:.1f}, y:{y:.1f}')

                if self.hold_counter >= self.hold_cycles:
                    self.current_wp_idx += 1
                    self.hold_counter = 0
            else:
                self.get_logger().info('Pattern complete — returning home.')
                self.phase = 'return'
                self.hold_counter = 0
            return

        if self.phase == 'return':
            hx, hy, hz, yaw = self.home_wp
            self.publish_setpoint(hx, hy, hz, yaw)
            dist = np.linalg.norm(self.current_position[:2] - np.array([hx, hy]))
            if self.hold_counter == 0:
                self.get_logger().info('Returning to home...')
            self.hold_counter += 1
            if dist < 0.5 or self.hold_counter > 5 * self.rate_hz:
                self.get_logger().info(f'Home reached (dist={dist:.2f}) — disarming.')
                self.disarm()
                self.phase = 'finished'
                self.destroy_timer(self.timer)
            return


def main(args=None):
    rclpy.init(args=args)
    node = LawnMowerOffboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt → shutdown')
    finally:
        if node.armed:
            node.disarm()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

