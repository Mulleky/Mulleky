#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import Float32
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import random


class HugeFieldGenerator(Node):
    def __init__(self):
        super().__init__('huge_field_generator')
        self.width = 75.0
        self.height = 75.0
        self.resolution = 1.0
        self.base_temperature = 20.0
        self.hotspot_amplitude = 10.0
        self.current_pos = None

        # 9 center points (each defines one 25x25 block)
        self.points = [
            (12.5, 12.5), (12.5, 37.5), (12.5, 62.5),
            (37.5, 12.5), (37.5, 37.5), (37.5, 62.5),
            (62.5, 12.5), (62.5, 37.5), (62.5, 62.5)
        ]
        self.hotspot_size = 25
        self.hotspot_generators = [
            self.generate_radial_field,
            self.generate_x_compress_field,
            self.generate_x_compress_tilt_field,
            self.generate_y_compress_field,
            self.generate_y_compress_tilt_field
        ]

        qos_viz = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        qos_px4 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.marker_pub = self.create_publisher(Marker, '/gaussian_field/huge', qos_viz)
        self.temp_pub = self.create_publisher(Float32, '/gaussian_field/huge/temperature', 10)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_px4)

        self.generate_field()
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)
        self.temp_timer = self.create_timer(0.1, self.publish_temperature)
        self.get_logger().info("Huge 75x75 field initialized")

    # -----------------------------
    # Field generation
    # -----------------------------
    def generate_field(self):
        """Generate 75x75 field with 5 random, non-repeating hotspots."""
        x = np.arange(0, self.width, self.resolution)
        y = np.arange(0, self.height, self.resolution)
        X, Y = np.meshgrid(x, y)

        # Base temperature field with noise
        field = self.base_temperature + np.random.normal(0, 0.5, X.shape)

        # Choose 5 random hotspot points
        hotspot_indices = random.sample(range(9), 5)
        # Randomly assign one of each unique hotspot generator
        selected_generators = random.sample(self.hotspot_generators, 5)
        self.hotspot_assignments = {}

        for idx, generator_fn in zip(hotspot_indices, selected_generators):
            cx, cy = self.points[idx]
            self.hotspot_assignments[(cx, cy)] = generator_fn.__name__

            # Create a 25x25 hotspot around this center
            i_start = int(cx - 12.5)
            j_start = int(cy - 12.5)
            i_end = i_start + 25
            j_end = j_start + 25

            subX = np.arange(i_start, i_end, self.resolution)
            subY = np.arange(j_start, j_end, self.resolution)
            subX, subY = np.meshgrid(subX, subY)
            hotspot = generator_fn(subX, subY, cx, cy)

            field[j_start:j_end, i_start:i_end] = hotspot

        self.temperature_field = field
        self.X, self.Y = X, Y
        self.temp_min = float(np.min(field))
        self.temp_max = float(np.max(field))

        # Log which hotspot was assigned where
        self.get_logger().info(f"Field generated with 5 unique hotspots:")
        for (cx, cy), name in self.hotspot_assignments.items():
            self.get_logger().info(f" - Point ({cx}, {cy}) → {name}")
        self.get_logger().info(f"Temp range: [{self.temp_min:.2f}, {self.temp_max:.2f}]°C")

    # -----------------------------
    # Hotspot generators
    # -----------------------------
    def generate_radial_field(self, X, Y, cx, cy):
        sigma = 5.0
        gaussian = np.exp(-((X - cx)**2 + (Y - cy)**2) / (2 * sigma**2))
        return self.base_temperature + self.hotspot_amplitude * gaussian

    def generate_x_compress_field(self, X, Y, cx, cy):
        sigma_x, sigma_y = 2.5, 7.0
        gaussian = np.exp(-((X - cx)**2 / (2 * sigma_x**2) + (Y - cy)**2 / (2 * sigma_y**2)))
        return self.base_temperature + self.hotspot_amplitude * gaussian

    def generate_x_compress_tilt_field(self, X, Y, cx, cy):
        sigma_x, sigma_y = 2.5, 7.0
        theta = np.pi / 4.0
        X_rot = (X - cx) * np.cos(theta) + (Y - cy) * np.sin(theta)
        Y_rot = -(X - cx) * np.sin(theta) + (Y - cy) * np.cos(theta)
        gaussian = np.exp(-(X_rot**2 / (2 * sigma_x**2) + Y_rot**2 / (2 * sigma_y**2)))
        return self.base_temperature + self.hotspot_amplitude * gaussian

    def generate_y_compress_field(self, X, Y, cx, cy):
        sigma_x, sigma_y = 7.0, 2.5
        gaussian = np.exp(-((X - cx)**2 / (2 * sigma_x**2) + (Y - cy)**2 / (2 * sigma_y**2)))
        return self.base_temperature + self.hotspot_amplitude * gaussian

    def generate_y_compress_tilt_field(self, X, Y, cx, cy):
        sigma_x, sigma_y = 7.0, 2.5
        theta = np.pi / 4.0
        X_rot = (X - cx) * np.cos(theta) + (Y - cy) * np.sin(theta)
        Y_rot = -(X - cx) * np.sin(theta) + (Y - cy) * np.cos(theta)
        gaussian = np.exp(-(X_rot**2 / (2 * sigma_x**2) + Y_rot**2 / (2 * sigma_y**2)))
        return self.base_temperature + self.hotspot_amplitude * gaussian

    # -----------------------------
    # ROS utilities
    # -----------------------------
    def sample_field(self, x, y):
        if x < 0 or x > self.width or y < 0 or y > self.height:
            return self.base_temperature
        x_grid = np.arange(0, self.width, self.resolution)
        y_grid = np.arange(0, self.height, self.resolution)
        i = np.searchsorted(x_grid, x) - 1
        j = np.searchsorted(y_grid, y) - 1
        if i < 0 or i >= len(x_grid)-1 or j < 0 or j >= len(y_grid)-1:
            return self.base_temperature
        x0, x1 = x_grid[i], x_grid[i+1]
        y0, y1 = y_grid[j], y_grid[j+1]
        wx = (x - x0) / (x1 - x0)
        wy = (y - y0) / (y1 - y0)
        temp = (
            (1-wx)*(1-wy)*self.temperature_field[j,i] +
            wx*(1-wy)*self.temperature_field[j,i+1] +
            (1-wx)*wy*self.temperature_field[j+1,i] +
            wx*wy*self.temperature_field[j+1,i+1]
        )
        return float(temp)

    def odom_callback(self, msg):
        self.current_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])

    def publish_temperature(self):
        if self.current_pos is None:
            return
        temp = self.sample_field(self.current_pos[0], self.current_pos[1])
        msg = Float32()
        msg.data = temp
        self.temp_pub.publish(msg)

    def publish_visualization(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "huge_field"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = 0.1
        marker.pose.orientation.w = 1.0
        rows, cols = self.X.shape
        for i in range(rows):
            for j in range(cols):
                temp = self.temperature_field[i, j]
                p = Point()
                p.x, p.y, p.z = float(self.X[i, j]), float(self.Y[i, j]), -0.1
                marker.points.append(p)
                normalized = (temp - self.temp_min) / (self.temp_max - self.temp_min)
                color = ColorRGBA()
                color.a = 0.8
                if normalized < 0.5:
                    color.r = normalized * 2.0
                    color.g = 0.0
                    color.b = 1.0 - normalized * 2.0
                else:
                    color.r = 1.0
                    color.g = (normalized - 0.5) * 2.0
                    color.b = 0.0
                marker.colors.append(color)
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = HugeFieldGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


