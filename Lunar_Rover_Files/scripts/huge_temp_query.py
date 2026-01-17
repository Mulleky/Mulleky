#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import csv
import os
import time
import glob
import re


class HugeFieldQuery(Node):
    def __init__(self):
        super().__init__('huge_field_query')

        # Set up temperature subscriber
        self.temperature = None
        self.create_subscription(
            Float32,
            '/gaussian_field/huge/temperature',
            self.temp_callback,
            10
        )

        # Set up odometry subscriber
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )

        # Store position info
        self.current_x = None
        self.current_y = None
        self.position_received = False

        # Data logging setup
        self.log_dir = "/home/carlos/DREAMS Lab/Aquattic drone"
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Find next available file number
        next_number = self.get_next_file_number()
        self.csv_path = os.path.join(self.log_dir, f"huge_field_data_{next_number}.csv")

        # Create CSV file with header
        with open(self.csv_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp (s)", "X (m)", "Y (m)", "Temperature (°C)"])

        # Timer for automatic logging (every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.log_data)

        self.get_logger().info("Huge Field Temperature Query initialized")
        self.get_logger().info(f"Logging data to: {self.csv_path}")
        self.get_logger().info("Subscribing to /gaussian_field/huge/temperature")

    def get_next_file_number(self):
        """Find the next available file number by scanning existing CSV files."""
        # Pattern to match huge_field_data_#.csv files
        pattern = os.path.join(self.log_dir, "huge_field_data_*.csv")
        existing_files = glob.glob(pattern)
        
        if not existing_files:
            return 1
        
        # Extract numbers from filenames
        numbers = []
        for filepath in existing_files:
            filename = os.path.basename(filepath)
            match = re.search(r'huge_field_data_(\d+)\.csv', filename)
            if match:
                numbers.append(int(match.group(1)))
        
        # Return the next number (max + 1)
        return max(numbers) + 1 if numbers else 1

    def temp_callback(self, msg):
        """Receive temperature from huge field."""
        self.temperature = msg.data

    def odom_callback(self, msg):
        """Receive rover position from PX4."""
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        if not self.position_received:
            self.position_received = True

    def log_data(self):
        """Automatically record (X, Y, Temperature) to CSV."""
        if not self.position_received or self.temperature is None:
            return  # Wait until both position and temperature are available

        timestamp = time.time()
        with open(self.csv_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, self.current_x, self.current_y, self.temperature])

    def display(self):
        """Display current temperature and position."""
        if not self.position_received:
            print("\nWaiting for rover position from /fmu/out/vehicle_odometry...")
            print("(Make sure PX4 simulation is running)")
            return

        print(f"\nRover Position: ({self.current_x:.2f}, {self.current_y:.2f}) m\n")

        if self.temperature is not None:
            print(f"🌡️  Huge Field Temperature: {self.temperature:.2f} °C\n")
        else:
            print("Waiting for temperature data from /gaussian_field/huge/temperature...\n")


def main(args=None):
    rclpy.init(args=args)
    node = HugeFieldQuery()

    # Run ROS spin in background
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    time.sleep(1)
    print("\nHuge Field Temperature Query")
    print("Press ENTER to show current rover temperature in the huge field")
    print("Type 'q' or 'quit' to exit\n")

    try:
        while rclpy.ok():
            user_input = input("").strip()
            if user_input.lower() in ['q', 'quit', 'exit']:
                print("\nExiting...\n")
                break
            node.display()
    except KeyboardInterrupt:
        print("\nInterrupted. Exiting...\n")
    except EOFError:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()