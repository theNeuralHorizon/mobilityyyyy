"""Start Gazebo Harmonic with the judge's world and the ROS-Gazebo clock bridge.

Usage:
  # Headless (WSL2 / CI / SSH — default):
  ros2 launch artpark_bringup sim.launch.py

  # With GUI (Ubuntu desktop with GPU):
  ros2 launch artpark_bringup sim.launch.py headless:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    grid_world_share = get_package_share_directory('grid_world')
    world_path = os.path.join(grid_world_share, 'worlds', 'grid_world_FINAL.sdf')

    gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(grid_world_share),
    )

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    # ---- HEADLESS mode ----
    # -s: server-only (no GUI). --headless-rendering: sensor rendering via
    # Mesa llvmpipe. QT_QPA_PLATFORM=offscreen: prevent display-not-found crash.
    gz_sim_headless = ExecuteProcess(
        condition=IfCondition(headless),
        cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', '-v', '3', world],
        output='screen',
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
            'QT_QPA_PLATFORM': 'offscreen',
        },
    )

    # ---- GUI mode ----
    # No -s flag → Gazebo opens its GUI window.
    # QT_QPA_PLATFORM=xcb forces X11 (Wayland breaks Ogre2 GLX window creation).
    # Do NOT set LIBGL_ALWAYS_SOFTWARE here — let the real GPU render the GUI.
    gz_sim_gui = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=['gz', 'sim', '-r', '-v', '3', world],
        output='screen',
        additional_env={
            'QT_QPA_PLATFORM': 'xcb',
        },
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_path,
                              description='Path to the SDF world'),
        DeclareLaunchArgument('headless', default_value='true',
                              description='Run Gazebo in headless mode (no GUI). '
                              'Set to false for desktop testing with GUI.'),
        gz_resource,
        gz_sim_headless, gz_sim_gui, clock_bridge,
    ])
