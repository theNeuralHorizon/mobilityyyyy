"""Spawn the mini_r1 rover into an already-running Gazebo sim.

Run after `gz sim -r grid_world_FINAL.sdf` is up. This launch publishes TF
from the xacro URDF, bridges the Gazebo sensor/command topics into ROS, and
spawns the robot at the solid-green START tile with yaw configurable via
argument.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('mini_r1_v1_description')
    xacro = PathJoinSubstitution([pkg, 'urdf', 'mini_r1.urdf.xacro'])

    spawn_x   = LaunchConfiguration('spawn_x')
    spawn_y   = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    robot_description = ParameterValue(
        Command(['xacro ', xacro, ' use_control:=false']),
        value_type=str,
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Delay the spawn by 3s so robot_state_publisher has published the
    # /robot_description topic before 'create' tries to subscribe to it.
    spawn = TimerAction(period=3.0, actions=[ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-topic', 'robot_description',
            '-name', 'mini_r1',
            ['-x', '=', spawn_x],
            ['-y', '=', spawn_y],
            '-z=0.07',
            ['-Y', '=', spawn_yaw],
        ],
        output='screen',
    )])

    # Belt-and-suspenders teleport to correct pose after spawn.
    fixup_pose = TimerAction(period=6.0, actions=[
        ExecuteProcess(
            cmd=['gz', 'service', '-s', '/world/artpark_arena/set_pose',
                 '--reqtype', 'gz.msgs.Pose',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000',
                 '--req', ['name: "mini_r1", position: {x: ', spawn_x,
                           ', y: ', spawn_y, ', z: 0.07}, orientation: {z: 0, w: 1}']],
            output='screen',
        ),
    ])

    # Topic bridges: Gazebo <-> ROS 2
    # All sensor topics now use absolute paths in the URDF, so they match
    # directly without needing model-namespace prefixing.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            # Commands: ROS -> GZ
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Sensor / state: GZ -> ROS
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
                              description='Yaw at spawn'),
        rsp, spawn, bridge, fixup_pose,
    ])
