#!/usr/bin/env python3
"""
Photogrammetry-optimized scan pattern with stop-and-shoot capability.
Integrates rover movement, camera aiming, and image capture coordination.
"""
import math
import os
import json
from enum import Enum
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Image
from px4_msgs.msg import (
    VehicleOdometry,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)


class State(Enum):
    WAIT_FOR_ODOM = 0
    START_OFFBOARD = 1
    MOVING_TO_WAYPOINT = 2
    STOPPED_AIMING_CAMERA = 3
    CAPTURING_IMAGE = 4
    FINISHED = 5


class PhotogrammetryScanNode(Node):
    """
    Stop-and-shoot photogrammetry scan with optimized photo positions.
    
    Features:
    - Stops at each photo waypoint
    - Aims camera at cylinder center
    - Waits for camera to stabilize
    - Triggers image save
    - Logs camera poses for Metashape
    """

    def __init__(self):
        super().__init__('photogrammetry_scan_node')

        # ----------------
        # Parameters
        # ----------------
        # Cylinder location (in your non-standard Gazebo frame: X=North, Y=West)
        self.declare_parameter('cylinder_sdf_x', 5.0)  # North in your SDF
        self.declare_parameter('cylinder_sdf_y', 0.0)  # West in your SDF
        self.declare_parameter('cylinder_height', 1.0)  # meters
        
        # Home position override (if you want to set manually instead of using first odom)
        self.declare_parameter('use_manual_home', False)
        self.declare_parameter('manual_home_north', 0.0)
        self.declare_parameter('manual_home_east', 0.0)

        # Photogrammetry scan parameters
        self.declare_parameter('num_circles', 3)
        self.declare_parameter('photos_per_circle', 36)
        self.declare_parameter('circle_radii', [4.0, 6.0, 8.0])  # meters from cylinder
        self.declare_parameter('clockwise', True)

        # Stop-and-shoot timing
        self.declare_parameter('position_tolerance', 0.3)  # meters
        self.declare_parameter('settle_time', 2.0)  # seconds to wait after stopping
        self.declare_parameter('camera_aim_time', 1.0)  # seconds for camera to aim
        
        # Camera parameters
        self.declare_parameter('camera_offset_z', 0.5)  # camera height above base_link
        
        # Output paths
        self.declare_parameter('output_dir', '/home/carlos/ros2_ws/src/lunar_rover/photogrammetry_data')
        self.declare_parameter('session_name', datetime.now().strftime('%Y%m%d_%H%M%S'))

        # Read parameters
        cyl_sdf_x = float(self.get_parameter('cylinder_sdf_x').value)
        cyl_sdf_y = float(self.get_parameter('cylinder_sdf_y').value)
        self.cylinder_height = float(self.get_parameter('cylinder_height').value)
        
        # Convert SDF coords (X=North, Y=West) to NED (North, East)
        # Your SDF: Y positive = West, so East = -Y
        self.cylinder_ned = (cyl_sdf_x, -cyl_sdf_y)  # (North, East)
        
        self.num_circles = int(self.get_parameter('num_circles').value)
        self.photos_per_circle = int(self.get_parameter('photos_per_circle').value)
        self.circle_radii = list(self.get_parameter('circle_radii').value)
        self.clockwise = bool(self.get_parameter('clockwise').value)
        
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.camera_aim_time = float(self.get_parameter('camera_aim_time').value)
        
        self.camera_offset_z = float(self.get_parameter('camera_offset_z').value)
        
        self.output_dir = str(self.get_parameter('output_dir').value)
        session_name = str(self.get_parameter('session_name').value)
        self.session_dir = os.path.join(self.output_dir, session_name)
        
        self.use_manual_home = bool(self.get_parameter('use_manual_home').value)
        self.manual_home_north = float(self.get_parameter('manual_home_north').value)
        self.manual_home_east = float(self.get_parameter('manual_home_east').value)
        
        # Create output directory
        os.makedirs(self.session_dir, exist_ok=True)
        
        # ----------------
        # State
        # ----------------
        self.state = State.WAIT_FOR_ODOM
        self.odom_received = False
        self.current_position = [0.0, 0.0, 0.0]  # NED
        self.current_yaw = 0.0
        self.offboard_set = False
        self.armed = False
        
        self.home_xy_ned = None
        self.photo_waypoints = []  # List of (north, east, photo_id)
        self.current_wp_idx = 0
        
        self.stop_timer = None
        self.capture_timer = None
        
        # Photo capture tracking
        self.photo_count = 0
        self.camera_poses = []  # List of pose dicts for export
        self.last_image_timestamp = None
        
        # ----------------
        # QoS
        # ----------------
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # ----------------
        # Publishers
        # ----------------
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10
        )
        self.camera_yaw_pub = self.create_publisher(
            Float64, '/cmd_pos', 10
        )
        self.capture_trigger_pub = self.create_publisher(
            Bool, '/camera/capture_trigger', 10
        )
        
        # ----------------
        # Subscribers
        # ----------------
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_callback, qos_best_effort
        )
        self.image_sub = self.create_subscription(
            Image,
            '/world/rover_lunar/model/r1_rover_camera_0/link/camera_link/sensor/camera_sensor/image',
            self.image_callback, 10
        )
        
        # ----------------
        # Timers
        # ----------------
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(
            f"PhotogrammetryScanNode initialized\n"
            f"  Cylinder NED: (N={self.cylinder_ned[0]:.2f}, E={self.cylinder_ned[1]:.2f})\n"
            f"  Circles: {self.num_circles}, Photos/circle: {self.photos_per_circle}\n"
            f"  Radii: {self.circle_radii}\n"
            f"  Session: {self.session_dir}"
        )

    # --------------------
    # Utilities
    # --------------------
    
    @staticmethod
    def yaw_from_quaternion(q):
        w, x, y, z = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    @staticmethod
    def wrap_to_pi(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi
    
    def calculate_camera_yaw_for_target(self):
        """Calculate camera yaw (relative to rover) to point at cylinder center."""
        cx, cy = self.cylinder_ned
        rx, ry = self.current_position[0], self.current_position[1]
        
        # World yaw to target (NED: atan2(East, North))
        target_world_yaw = math.atan2(cy - ry, cx - rx)
        
        # Camera yaw relative to rover body
        camera_yaw = self.wrap_to_pi(target_world_yaw - self.current_yaw)
        
        return camera_yaw
    
    def distance_to_waypoint(self, north, east):
        dn = north - self.current_position[0]
        de = east - self.current_position[1]
        return math.sqrt(dn*dn + de*de)
    
    # --------------------
    # Callbacks
    # --------------------
    
    def odom_callback(self, msg: VehicleOdometry):
        self.current_position = [msg.position[0], msg.position[1], msg.position[2]]
        self.current_yaw = self.yaw_from_quaternion(msg.q)
        self.odom_received = True
        
        # Set home position
        if self.home_xy_ned is None:
            if self.use_manual_home:
                # Use manually specified home position
                self.home_xy_ned = (self.manual_home_north, self.manual_home_east)
                self.get_logger().info(
                    f"Using manual home position: (N={self.manual_home_north:.4f}, E={self.manual_home_east:.4f})"
                )
            else:
                # Collect samples during initialization to get stable home position
                if len(self.odom_samples) < self.odom_samples_needed:
                    self.odom_samples.append((self.current_position[0], self.current_position[1]))
                    
                    if len(self.odom_samples) == self.odom_samples_needed:
                        # Average the samples for stable home position
                        avg_north = sum(s[0] for s in self.odom_samples) / len(self.odom_samples)
                        avg_east = sum(s[1] for s in self.odom_samples) / len(self.odom_samples)
                        self.home_xy_ned = (avg_north, avg_east)
                        self.get_logger().info(
                            f"Home position established: (N={avg_north:.4f}, E={avg_east:.4f}) "
                            f"from {self.odom_samples_needed} samples"
                        )
    
    def image_callback(self, msg: Image):
        """Track when images arrive (for debugging/logging)."""
        self.last_image_timestamp = self.get_clock().now()
    
    # --------------------
    # PX4 Commands
    # --------------------
    
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_mode_pub.publish(msg)
    
    def publish_trajectory_setpoint(self, north, east, down, yaw):
        msg = TrajectorySetpoint()
        msg.position = [float(north), float(east), float(down)]
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
        self.get_logger().info("ARM")
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.armed = True
    
    def disarm(self):
        self.get_logger().info("DISARM")
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.armed = False
    
    def set_offboard_mode(self):
        self.get_logger().info("Setting OFFBOARD mode")
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.offboard_set = True
    
    def publish_camera_yaw(self, yaw_rad):
        """Command camera yaw joint."""
        msg = Float64()
        msg.data = float(yaw_rad)
        self.camera_yaw_pub.publish(msg)
    
    def trigger_image_capture(self):
        """Signal that an image should be captured (for integration with image_saver)."""
        msg = Bool()
        msg.data = True
        self.capture_trigger_pub.publish(msg)
        self.get_logger().info(f"📸 Image capture triggered (photo #{self.photo_count})")
    
    # --------------------
    # Pattern Generation
    # --------------------
    
    def generate_photo_waypoints(self):
        """Generate waypoints optimized for photogrammetry."""
        cx, cy = self.cylinder_ned
        waypoints = []
        
        angle_step = 360.0 / self.photos_per_circle  # degrees
        direction = -1 if self.clockwise else 1
        
        photo_id = 0
        
        for circle_idx, radius in enumerate(self.circle_radii):
            self.get_logger().info(f"Generating circle {circle_idx+1}/{self.num_circles} at radius {radius}m")
            
            for i in range(self.photos_per_circle):
                angle_deg = i * angle_step * direction
                angle_rad = math.radians(angle_deg)
                
                # Position in NED
                north = cx + radius * math.cos(angle_rad)
                east = cy + radius * math.sin(angle_rad)
                
                waypoints.append({
                    'north': north,
                    'east': east,
                    'photo_id': photo_id,
                    'circle': circle_idx,
                    'radius': radius,
                    'angle_deg': angle_deg
                })
                photo_id += 1
        
        self.get_logger().info(f"Generated {len(waypoints)} photo waypoints")
        return waypoints
    
    # --------------------
    # State Machine
    # --------------------
    
    def control_loop(self):
        """Main state machine."""
        
        if self.state == State.WAIT_FOR_ODOM:
            if not self.odom_received:
                # Only log occasionally to avoid spam
                if not hasattr(self, '_wait_log_counter'):
                    self._wait_log_counter = 0
                self._wait_log_counter += 1
                if self._wait_log_counter % 100 == 0:  # Every 5 seconds at 20Hz
                    self.get_logger().info("Waiting for odometry...")
                return
            
            if self.home_xy_ned is None:
                if not hasattr(self, '_home_log_counter'):
                    self._home_log_counter = 0
                self._home_log_counter += 1
                if self._home_log_counter % 100 == 0:
                    self.get_logger().info(
                        f"Waiting for home position (collected {len(self.odom_samples)}/{self.odom_samples_needed} samples)..."
                    )
                return
            
            self.photo_waypoints = self.generate_photo_waypoints()
            self.current_wp_idx = 0
            
            self.get_logger().info(f"Home: (N={self.home_xy_ned[0]:.2f}, E={self.home_xy_ned[1]:.2f})")
            self.get_logger().info(f"Starting photogrammetry scan with {len(self.photo_waypoints)} waypoints")
            
            self.state = State.START_OFFBOARD
            return
        
        # Always publish offboard mode when active
        if self.state in (State.START_OFFBOARD, State.MOVING_TO_WAYPOINT, 
                          State.STOPPED_AIMING_CAMERA, State.CAPTURING_IMAGE):
            self.publish_offboard_control_mode()
        
        if self.state == State.START_OFFBOARD:
            n, e, d = self.current_position
            self.publish_trajectory_setpoint(n, e, d, self.current_yaw)
            
            if not self.armed:
                self.arm()
            if not self.offboard_set:
                self.set_offboard_mode()
            
            if self.armed and self.offboard_set:
                self.get_logger().info("✅ OFFBOARD + ARMED, starting scan")
                self.state = State.MOVING_TO_WAYPOINT
            else:
                # Log status occasionally
                if not hasattr(self, '_offboard_log_counter'):
                    self._offboard_log_counter = 0
                self._offboard_log_counter += 1
                if self._offboard_log_counter % 100 == 0:
                    self.get_logger().info(
                        f"Waiting for OFFBOARD mode (armed={self.armed}, offboard_set={self.offboard_set})"
                    )
            return
        
        if self.state == State.MOVING_TO_WAYPOINT:
            if self.current_wp_idx >= len(self.photo_waypoints):
                self.finish_scan()
                return
            
            wp = self.photo_waypoints[self.current_wp_idx]
            target_n = wp['north']
            target_e = wp['east']
            target_d = self.current_position[2]  # maintain altitude
            
            # Yaw to face cylinder while moving
            cx, cy = self.cylinder_ned
            yaw_to_cylinder = math.atan2(cy - self.current_position[1], 
                                          cx - self.current_position[0])
            
            self.publish_trajectory_setpoint(target_n, target_e, target_d, yaw_to_cylinder)
            
            dist = self.distance_to_waypoint(target_n, target_e)
            
            if dist < self.position_tolerance:
                self.get_logger().info(
                    f"📍 Reached waypoint {self.current_wp_idx+1}/{len(self.photo_waypoints)} "
                    f"(Circle {wp['circle']+1}, Photo {wp['photo_id']+1})"
                )
                self.state = State.STOPPED_AIMING_CAMERA
                
                # Start settle timer
                self.stop_timer = self.create_timer(self.settle_time, self.on_settled)
            return
        
        if self.state == State.STOPPED_AIMING_CAMERA:
            # Hold position and aim camera
            wp = self.photo_waypoints[self.current_wp_idx]
            n, e, d = self.current_position
            
            # Keep rover facing cylinder
            cx, cy = self.cylinder_ned
            yaw_to_cylinder = math.atan2(cy - e, cx - n)
            self.publish_trajectory_setpoint(n, e, d, yaw_to_cylinder)
            
            # Aim camera at cylinder
            camera_yaw = self.calculate_camera_yaw_for_target()
            self.publish_camera_yaw(camera_yaw)
            return
        
        if self.state == State.CAPTURING_IMAGE:
            # Continue holding position during capture
            n, e, d = self.current_position
            cx, cy = self.cylinder_ned
            yaw_to_cylinder = math.atan2(cy - e, cx - n)
            self.publish_trajectory_setpoint(n, e, d, yaw_to_cylinder)
            return
        
        if self.state == State.FINISHED:
            return
    
    def on_settled(self):
        """Called after rover has settled at waypoint."""
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None
        
        self.get_logger().info("🎯 Rover settled, aiming camera...")
        
        # Aim camera precisely
        camera_yaw = self.calculate_camera_yaw_for_target()
        self.publish_camera_yaw(camera_yaw)
        
        # Wait for camera to aim, then capture
        self.capture_timer = self.create_timer(self.camera_aim_time, self.on_camera_aimed)
    
    def on_camera_aimed(self):
        """Called after camera has aimed at target."""
        if self.capture_timer:
            self.capture_timer.cancel()
            self.capture_timer = None
        
        self.state = State.CAPTURING_IMAGE
        
        # Log camera pose
        wp = self.photo_waypoints[self.current_wp_idx]
        pose = {
            'photo_id': wp['photo_id'],
            'circle': wp['circle'],
            'radius': wp['radius'],
            'angle_deg': wp['angle_deg'],
            'position_ned': {
                'north': float(self.current_position[0]),
                'east': float(self.current_position[1]),
                'down': float(self.current_position[2])
            },
            'camera_position_ned': {
                'north': float(self.current_position[0]),
                'east': float(self.current_position[1]),
                'down': float(self.current_position[2] - self.camera_offset_z)
            },
            'rover_yaw_rad': float(self.current_yaw),
            'camera_yaw_relative_rad': float(self.calculate_camera_yaw_for_target()),
            'timestamp': self.get_clock().now().nanoseconds
        }
        self.camera_poses.append(pose)
        
        # Trigger capture
        self.trigger_image_capture()
        self.photo_count += 1
        
        # Move to next waypoint after brief delay
        self.create_timer(0.5, self.move_to_next_waypoint, oneshot=True)
    
    def move_to_next_waypoint(self):
        """Advance to next waypoint."""
        self.current_wp_idx += 1
        self.state = State.MOVING_TO_WAYPOINT
        
        if self.current_wp_idx < len(self.photo_waypoints):
            wp = self.photo_waypoints[self.current_wp_idx]
            self.get_logger().info(
                f"➡️  Moving to next waypoint: Circle {wp['circle']+1}, "
                f"Photo {wp['photo_id']+1}/{len(self.photo_waypoints)}"
            )
        else:
            self.finish_scan()
    
    def finish_scan(self):
        """Complete scan and save metadata."""
        self.get_logger().info("🎉 Photogrammetry scan complete!")
        self.get_logger().info(f"Total photos: {self.photo_count}")
        
        # Hold final position
        n, e, d = self.current_position
        self.publish_trajectory_setpoint(n, e, d, self.current_yaw)
        
        # Save camera poses
        poses_file = os.path.join(self.session_dir, 'camera_poses.json')
        with open(poses_file, 'w') as f:
            json.dump({
                'session': str(self.get_parameter('session_name').value),
                'cylinder_ned': self.cylinder_ned,
                'num_photos': self.photo_count,
                'poses': self.camera_poses
            }, f, indent=2)
        
        self.get_logger().info(f"📝 Camera poses saved to: {poses_file}")
        
        # Disarm
        if self.armed:
            self.disarm()
        
        self.state = State.FINISHED


def main(args=None):
    rclpy.init(args=args)
    node = PhotogrammetryScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()