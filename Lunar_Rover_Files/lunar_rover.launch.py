#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Package directories ---
    sim_pkg_dir = get_package_share_directory('lunar_rover')
    control_pkg_dir = get_package_share_directory('control')

    # --- Paths ---
    images_dir = "/home/carlos/ros2_ws/src/lunar_rover/sim_images"

    # We keep this for RViz and any ROS-side use, but PX4 will launch Gazebo.
    world_file = os.path.join(sim_pkg_dir, 'worlds', 'rover_lunar.sdf')

    urdf_file = os.path.join(control_pkg_dir, 'urdf', 'r1_rover_with_camera.urdf')
    rviz_config = os.path.join(control_pkg_dir, 'config', 'rover_viz.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # --- Gazebo resource paths for custom models ---
    # Make sure this includes your PX4 custom_terrain and rover models.
    px4_models_dir = "/home/carlos/PX4-Autopilot/Tools/simulation/gz/models"

    gz_existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    ign_existing = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")

    gz_value = f"{gz_existing}:{px4_models_dir}" if gz_existing else px4_models_dir
    ign_value = f"{ign_existing}:{px4_models_dir}" if ign_existing else px4_models_dir

    set_gz_resources = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gz_value,
    )

    set_ign_resources = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=ign_value,
    )

    # Optional but consistent with manual tests
    set_gz_partition = SetEnvironmentVariable(
        name="GZ_PARTITION",
        value="px4",
    )

    # --- MicroXRCEAgent ---
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # --- PX4 SITL (PX4 launches Gazebo + rover_lunar world) ---
    px4_sitl = ExecuteProcess(
        cmd=[
            '/home/carlos/PX4-Autopilot/build/px4_sitl_default/bin/px4'
        ],
        additional_env={
            # IMPORTANT: use the same ID that works with `make px4_sitl gz_r1_rover`
            'PX4_SYS_AUTOSTART': '4022',  # airframe ID
            'PX4_SIMULATOR': 'gz',
            'PX4_SIM_MODEL': 'r1_rover_camera',       # later you can change to 'r1_rover_camera'
            'PX4_GZ_WORLD': 'rover_lunar',            # If i dont give it a path, it uses the worlds inside PX4
            'PX4_GZ_MODEL_POSE': '-7.0,2,-14.0,0,0,0',
            'GZ_PARTITION': 'px4',
        },
        output='screen'
    )

    # --- TF: world -> odom ---
    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen'
    )

    # --- TF: odom -> field ---
    odom_to_field = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_field',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'field'],
        output='screen'
    )

    # --- Rover monitor (odom -> base_link etc.) ---
    rover_monitor = Node(
        package='control',
        executable='rover_monitor.py',
        name='rover_monitor',
        output='screen',
        emulate_tty=True
    )

    # --- Scan pattern node (offboard control) ---
    # Delay to give PX4 + Gazebo + XRCE time to start
    scan_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='control',
                executable='test_pattern.py',
                name='scan_pattern_node',
                #name='photo_scan_pattern',
                output='screen',
                parameters=[{
                    'num_circles': 3,
                    'inner_radius': 5.0,
                    'radius_increment': 4.0,
                    'points_per_circle': 24,
                    'velocity': 1.0,
                    'position_tolerance': 1.0,
                    'hold_duration_s': 2.0,
                    'lookahead_m': 0.6,
                }]
            )
        ]
    )

    
    # --- Camera yaw controller: always look at scan-pattern center ---
    camera_yaw_controller = TimerAction(
        period=3.5,  # start slightly after scan pattern
        actions=[
            Node(
                package='control',
                executable='camera_yaw_spin.py',
                name='camera_yaw_look_at_center',
                output='screen',
                parameters=[{
                    # MUST match test_pattern center
                    'center_east_enu': 5.0,
                    'center_north_enu': 0.0,

                    # Joint command topic
                    'cmd_topic': (
                        '/world/rover_lunar/model/'
                        'r1_rover_camera_0/joint/'
                        'camera_yaw_joint/cmd_pos'
                    ),

                    # PX4 odometry
                    'odom_topic': '/fmu/out/vehicle_odometry',

                    # Control tuning
                    'publish_rate_hz': 30.0,
                    'max_yaw_rate_rad_s': 0.55,
                    'yaw_deadband_rad': 0.03,
                    'camera_yaw_offset_rad': 0.5,

                    # Debug
                    'debug': True,
                }]
            )
        ]
    )







    # --- ROS–Gazebo bridge for camera topics ---
    camera_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/rover_lunar/model/r1_rover_camera_0/link/camera_link/sensor/camera_sensor/image'
            '@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/rover_lunar/model/r1_rover_camera_0/link/camera_link/sensor/camera_sensor/camera_info'
            '@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # --- Robot State Publisher ---
    robot_state_publisher = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': robot_description,
                    'frame_prefix': '',
                    'publish_frequency': 1.0,
                }],
                output='screen'
            )
        ]
    )

    # --- Optional RViz ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    image_saver = Node(
        package='image_view',
        executable='image_saver',
        name='camera_image_saver',
        output='screen',
        remappings=[
            # ROS 2 image topic from your bridge
            ('image', '/world/rover_lunar/model/r1_rover_camera_0/link/camera_link/sensor/camera_sensor/image'),
            ('camera_info', '/world/rover_lunar/model/r1_rover_camera_0/link/camera_link/sensor/camera_sensor/camera_info'),
        ],
        parameters=[{
            'save_interval': 1.0, 
            # Save every image that arrives
            'save_all_image': True,
            # Absolute path + filename pattern
            'filename_format': os.path.join(images_dir, 'frame_%04d.png'),
        }]
    )

    # --- Rosbag2 (auto-name by date + run id, save to your folder) ---
    rosbag_dir = "/home/carlos/ros2_ws/src/lunar_rover/ros_bag"
    rosbag_timestamp = ExecuteProcess(
        cmd=['bash', '-lc', 'date +%Y%m%d_%H%M%S'],
        output='screen'
    )

    # Ensure rosbag directory exists
    make_rosbag_dir = ExecuteProcess(
        cmd=['bash', '-lc', f'mkdir -p "{rosbag_dir}"'],
        output='screen'
    )

    # Record camera + metadata topics into: /home/carlos/ros2_ws/src/lunar_rover/ros_bag/<TIMESTAMP>_run01
    rosbag_record = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            (
                f'TS=$(date +%Y%m%d_%H%M%S); '
                f'ros2 bag record -o "{rosbag_dir}/${{TS}}_run01" '
                f'/world/rover_lunar/model/r1_rover_camera_0/link/camera_link/sensor/camera_sensor/image '
                f'/world/rover_lunar/model/r1_rover_camera_0/link/camera_link/sensor/camera_sensor/camera_info '
                f'/tf /tf_static /odom'
            )
        ],
        output='screen'
    )

    rqt = ExecuteProcess(
    cmd=[
        'rqt',
        '--perspective-file',
        '/home/carlos/ros2_ws/src/lunar_rover/config/rover_monitor.perspective'
    ],
    output='screen'
    )


    # Start bagging a few seconds after sim start (topics exist)
    delayed_rosbag_record = TimerAction(
        period=8.0,
        actions=[make_rosbag_dir, rosbag_record]
    )

    # (Optional) Start saving a few seconds after sim start, so you skip
    # some initial junk frames or PX4 boot-up:
    delayed_image_saver = TimerAction(
        period=8.0,          # seconds
        actions=[image_saver]
    )

    delayed_rqt = TimerAction(
        period=9.0, 
        actions=[rqt]
    )


    return LaunchDescription([
        set_gz_resources,
        set_ign_resources,
        set_gz_partition,

        micro_xrce_agent,
        px4_sitl,

        world_to_odom,
        odom_to_field,
        rover_monitor,
        robot_state_publisher,
        scan_node,
        #camera_yaw_controller,
        camera_bridge,
        delayed_image_saver,
        delayed_rosbag_record,
        delayed_rqt,
        #rviz,  # uncomment when you want RViz
    ])




