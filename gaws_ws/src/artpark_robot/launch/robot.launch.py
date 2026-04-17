"""Spawn the artpark_bot into an already-running Gazebo sim.

Run after `gz sim -r grid_world_FINAL.sdf` is up. This launch publishes TF
from the xacro URDF, bridges the Gazebo sensor/command topics into ROS, and
spawns the robot at the solid-green START tile with yaw configurable via
argument.

Note on spawn args: the ros_gz_sim/create tool uses argparse which can
mis-parse negative values like `-x -1.35` (treats `-1.35` as a flag). We
work around this by either (a) using a wrapper bash command with `--` as a
separator, or (b) using ExecuteProcess with explicit argv so no shell
splitting happens. We go with (b) — most robust.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('artpark_robot')
    xacro = PathJoinSubstitution([pkg, 'urdf', 'artpark_bot.urdf.xacro'])

    spawn_x   = LaunchConfiguration('spawn_x')
    spawn_y   = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    robot_description = ParameterValue(Command(['xacro ', xacro]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Spawn via ExecuteProcess so argv is passed as-is (no shell re-parsing),
    # and use the `=` form for flags that take potentially-negative values.
    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-topic', 'robot_description',
            '-name', 'artpark_bot',
            ['-x', '=', spawn_x],   # → `-x=-1.35` — argparse-safe
            ['-y', '=', spawn_y],
            '-z=0.05',
            ['-Y', '=', spawn_yaw],
        ],
        output='screen',
    )

    # After the create process exits, call set_pose as a belt-and-suspenders
    # teleport. Re-snaps the robot to the intended world pose even if the
    # -x/-y flags were ignored (common Harmonic-gotcha with negative vals).
    fixup_pose = TimerAction(period=2.0, actions=[
        ExecuteProcess(
            cmd=['gz', 'service', '-s', '/world/artpark_arena/set_pose',
                 '--reqtype', 'gz.msgs.Pose',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000',
                 '--req', ['name: "artpark_bot", position: {x: ', spawn_x,
                           ', y: ', spawn_y, ', z: 0.05}, orientation: {z: 0, w: 1}']],
            output='screen',
        ),
    ])

    # Topic bridges: Gazebo ↔ ROS 2
    # Direction symbols (ros_gz_bridge v>=0.244):
    #   @  bidirectional (only when truly needed)
    #   [  GZ publisher → ROS subscriber (sensor data, odom, tf)
    #   ]  ROS publisher → GZ subscriber (commands)
    # Making these one-way avoids feedback loops where our ROS-side
    # publishers (robot_state_publisher etc.) re-publish to GZ and
    # confuse the physics engine.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            # Commands: ROS → GZ
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Sensor / state: GZ → ROS
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/front_cam/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/front_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/floor_cam/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/floor_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('spawn_x',   default_value='-1.35',
                              description='World x of the START tile (solid green)'),
        DeclareLaunchArgument('spawn_y',   default_value='1.80',
                              description='World y of the START tile (solid green)'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0',
                              description='Yaw at spawn; adjust after P0 screenshot'),
        rsp, spawn, bridge, fixup_pose,
    ])
