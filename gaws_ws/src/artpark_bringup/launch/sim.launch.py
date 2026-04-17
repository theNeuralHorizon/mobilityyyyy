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

    # Qt offscreen platform — prevents "could not connect to display" crash
    # when running without X11/Wayland (WSL2 headless, CI, SSH).
    qt_offscreen = SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='offscreen')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    # Headless mode: -s (server-only, no GUI) + --headless-rendering
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

    # GUI mode: no -s flag, runs with Gazebo GUI window.
    # Force software rendering to avoid Ogre2 GLX crashes on hybrid-GPU laptops.
    gz_sim_gui = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=['gz', 'sim', '-r', '--render-engine', 'ogre', '-v', '3', world],
        output='screen',
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
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
        gz_resource, qt_offscreen,
        gz_sim_headless, gz_sim_gui, clock_bridge,
    ])
