"""Master launch — sim + robot + perception + decision + logger.

Usage:
  ros2 launch artpark_bringup full_run.launch.py spawn_yaw:=0.0

Prereq: workspace is built and sourced, and the /front_cam topic is being
bridged from Gazebo to ROS (handled by robot.launch.py).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('artpark_bringup')
    robot   = FindPackageShare('artpark_robot')

    tag_yaml = PathJoinSubstitution([bringup, 'config', 'tag_label_map.yaml'])
    hsv_yaml = PathJoinSubstitution([bringup, 'config', 'hsv_thresholds.yaml'])
    nav_yaml = PathJoinSubstitution([bringup, 'config', 'navigation.yaml'])

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'sim.launch.py'])),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
    )

    # Give Gazebo time to finish booting before spawning the robot.
    # GUI mode needs ~10s for rendering pipeline init; headless is faster but
    # this delay is safe for both.
    robot_spawn = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([robot, 'launch', 'robot.launch.py'])),
            launch_arguments={
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            }.items(),
        )],
    )

    # apriltag detection now runs in-process inside apriltag_handler (OpenCV
    # ArUco with DICT_APRILTAG_36h11) — no separate apriltag_ros node.

    sim_time = {'use_sim_time': True}

    handler = Node(
        package='artpark_perception',
        executable='apriltag_handler',
        name='apriltag_handler',
        parameters=[tag_yaml, sim_time],
        output='screen',
    )

    floor = Node(
        package='artpark_perception',
        executable='floor_logo_detector',
        name='floor_logo_detector',
        parameters=[hsv_yaml, sim_time],
        output='screen',
    )

    obs = Node(
        package='artpark_perception',
        executable='obstacle_monitor',
        name='obstacle_monitor',
        parameters=[nav_yaml, sim_time],
        output='screen',
    )

    tile = Node(
        package='artpark_decision',
        executable='tile_tracker',
        name='tile_tracker',
        parameters=[{
            'spawn_world_x': LaunchConfiguration('spawn_x'),
            'spawn_world_y': LaunchConfiguration('spawn_y'),
            'use_sim_time': True,
        }],
        output='screen',
    )

    sm = Node(
        package='artpark_decision',
        executable='state_machine',
        name='state_machine',
        parameters=[nav_yaml, {
            'debug_hold': LaunchConfiguration('debug_hold'),
            'use_sim_time': True,
        }],
        output='screen',
    )

    logger = Node(
        package='artpark_logger',
        executable='logger_node',
        name='logger_node',
        parameters=[sim_time],
        output='screen',
    )

    # Perception + decision come up AFTER the robot (hence another delay).
    downstream = TimerAction(period=18.0, actions=[handler, floor, obs, tile, sm, logger])

    return LaunchDescription([
        DeclareLaunchArgument('spawn_x',   default_value='-1.35'),
        DeclareLaunchArgument('spawn_y',   default_value='1.80'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('debug_hold', default_value='false',
                              description='If true, the state machine stays in INIT and publishes no motion.'),
        DeclareLaunchArgument('headless', default_value='true',
                              description='Run Gazebo headless (no GUI). Set false for desktop testing.'),
        sim, robot_spawn, downstream,
    ])
