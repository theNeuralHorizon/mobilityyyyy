"""P0 verification run: sim + robot + apriltag_ros + teleop keyboard.

Drive the robot manually past every AprilTag, watch the printed (tag_id,
pose) pairs, then fill tag_label_map.yaml accordingly.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('artpark_bringup')
    robot   = FindPackageShare('artpark_robot')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'sim.launch.py']))
    )
    robot_spawn = TimerAction(period=4.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([robot, 'launch', 'robot.launch.py'])),
            launch_arguments={
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            }.items(),
        )
    ])

    # Run the in-process handler so you can watch /tag_event for id→position
    # mapping during the teleop drive.
    handler = Node(
        package='artpark_perception',
        executable='apriltag_handler',
        name='apriltag_handler',
        parameters=[PathJoinSubstitution([bringup, 'config', 'tag_label_map.yaml'])],
        output='screen',
    )

    # Teleop — instruct user to run `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
    # in a separate terminal. Not auto-launched because it needs TTY.

    return LaunchDescription([
        DeclareLaunchArgument('spawn_x', default_value='-1.35'),
        DeclareLaunchArgument('spawn_y', default_value='1.80'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        sim, robot_spawn,
        TimerAction(period=7.0, actions=[handler]),
    ])
