"""Launch the existing zone simulator and monitoring CheapStage viewer."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package = Path(get_package_share_directory('obst_avoidance'))
    viewer = ExecuteProcess(cmd=['ros2', 'run', 'obst_avoidance', 'cheap_viewer',
                                 '--startup-timeout', '60'], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='false', description='Also show Gazebo GUI'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch/env_zones.launch.py')),
            launch_arguments={'use_gui': LaunchConfiguration('use_gui'),
                              'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('start_sim')),
        ),
        RegisterEventHandler(OnProcessExit(target_action=viewer,
            on_exit=[EmitEvent(event=Shutdown(reason='Camera viewer closed'))])),
        viewer,
    ])
