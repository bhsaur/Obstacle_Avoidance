#!/usr/bin/env python3
"""Launch ArduPilot SITL + Gazebo Harmonic with the env_zones world, spawn
the Iris (with gimbal/camera), and bring up MAVROS so the drone can be
commanded (arm/takeoff/set_mode/velocity setpoints) over ROS 2 topics and
services.

This is env_forest.launch.py with its two world defaults pointed at
env_zones instead (see Step X in README): env_forest.sdf failed its own
audit -- 7 obstacles crammed into a single 28x10m cluster, untextured --
and env_zones.sdf is the evaluation world built to replace it. Kept as a
separate file rather than an argument override on env_forest.launch.py
so the safe path is the default one: a bare `ros2 launch obst_avoidance
env_zones.launch.py` with no extra flags loads the right world, instead
of requiring both world_path AND world_name to be remembered and passed
correctly every time.

world_name below MUST match <world name="..."> inside env_zones.sdf --
confirmed: env_zones.sdf declares <world name="env_zones">.

Composed the same way as seaweed_sim/iris_seaweed.launch.py:
  1. ros_gz_sim's gz_sim.launch.py       -> starts Gazebo with OUR world.
  2. ardupilot_gz_bringup's robots/iris.launch.py -> starts SITL and
     spawns the Iris into the already-running world at a given pose.
  3. mavros' apm.launch                  -> connects to SITL's MAVLink
     output so you can send flight commands.

Usage:
  ros2 launch obst_avoidance env_zones.launch.py
  ros2 launch obst_avoidance env_zones.launch.py spawn_x:=-3.0 spawn_y:=0.0
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import SetParameter


def generate_launch_description() -> LaunchDescription:
    pkg_obst_avoidance = get_package_share_directory("obst_avoidance")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_ardupilot_gz_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_mavros = get_package_share_directory("mavros")

    # Path to the world this package installs (see setup.py data_files).
    default_world_path = os.path.join(pkg_obst_avoidance, "worlds", "env_zones.sdf")

    # So Gazebo can resolve the Iris's package://ardupilot_gazebo/... mesh
    # URIs -- prepend its models dir to whatever GZ_SIM_RESOURCE_PATH is
    # already set to (same fix seaweed_sim's launch file applies).
    ardupilot_models_path = os.path.join(pkg_ardupilot_gazebo, "models")
    ardupilot_share_path = os.path.dirname(pkg_ardupilot_gazebo)
    # Also needed so package://obst_avoidance/models/... (our 640x480 gimbal
    # override, see models/iris_with_gimbal_640x480) resolves the same way.
    obst_avoidance_share_path = os.path.dirname(pkg_obst_avoidance)
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_resource_path = os.pathsep.join(filter(None, [
        ardupilot_models_path,
        ardupilot_share_path,
        obst_avoidance_share_path,
        existing_resource_path,
    ]))
    set_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", new_resource_path
    )

    # ---- Launch arguments -------------------------------------------------
    world_path_arg = DeclareLaunchArgument(
        "world_path",
        default_value=default_world_path,
        description="Absolute path to the Gazebo world SDF to load.",
    )
    # Must match the <world name="..."> inside the SDF -- it's used to
    # namespace the gz spawn service (/world/<world_name>/create).
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="env_zones",
        description="Name of the <world> in the SDF (for the spawn service).",
    )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="iris", description="Name for the spawned model."
    )
    # Spawn behind the tree line (trees start around x=2) so there's room
    # to take off and fly toward the obstacles.
    spawn_x_arg = DeclareLaunchArgument("spawn_x", default_value="-3.0")
    spawn_y_arg = DeclareLaunchArgument("spawn_y", default_value="0.0")
    # Spawn above the ground plane rather than right at its surface -- at
    # z=0.2 the landing-gear collision mesh starts in slight contact with
    # ground_plane, and the constant contact-resolution work each physics
    # step tanks the real-time factor. Falling a few cm onto solid ground
    # is cheaper than resolving interpenetration every step.
    spawn_z_arg = DeclareLaunchArgument("spawn_z", default_value="0.5")
    spawn_yaw_arg = DeclareLaunchArgument("spawn_yaw", default_value="0.0")

    use_mavros_arg = DeclareLaunchArgument(
        "use_mavros",
        default_value="true",
        description="Start MAVROS so the drone can be commanded over ROS 2.",
    )
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="udp://:14550@127.0.0.1:14550",
        description="MAVLink connection URL to ArduPilot SITL's mavlink output.",
    )
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Open the Gazebo client window. Set false to run headless "
        "(server-only) -- the camera sensor and physics are unaffected, but "
        "dropping the GUI's own rendering load raises the real-time factor.",
    )

    # ---- 1) Start Gazebo with our world ------------------------------------
    gz_flags = [
        LaunchConfiguration("world_path"),
        TextSubstitution(text=" -r -v4"),
    ]
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": gz_flags,
            "gz_version": "8",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": gz_flags + [TextSubstitution(text=" -s")],
            "gz_version": "8",
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_gui")),
    )

    # Use our 640x480 override of iris_with_gimbal instead of upstream's
    # 640x4800 camera sensor (see models/iris_with_gimbal_640x480).
    #
    # NOTE: ardupilot_gz_bringup's robots/iris.launch.py hardcodes its own
    # sdf_file internally (does not expose it as a passthrough argument) --
    # so we include robots/robot.launch.py directly instead, which is what
    # iris.launch.py itself delegates to and DOES declare "sdf_file" as a
    # real launch argument. All other args below match iris.launch.py's own
    # defaults; we only override what needs to differ.
    iris_sdf_file = os.path.join(
        pkg_obst_avoidance, "models", "iris_with_gimbal_640x480", "model.sdf"
    )
    # robot.launch.py's own "defaults" default omits dds_use_ns.parm --
    # iris.launch.py normally overrides it with this exact set; replicate
    # that here so bypassing iris.launch.py doesn't change SITL behavior.
    sitl_defaults = ",".join([
        os.path.join(pkg_ardupilot_sitl, "config", "default_params", "copter.parm"),
        os.path.join(pkg_ardupilot_gazebo, "config", "gazebo-iris-gimbal.parm"),
        os.path.join(pkg_ardupilot_sitl, "config", "default_params", "dds_udp.parm"),
        os.path.join(pkg_ardupilot_sitl, "config", "default_params", "dds_use_ns.parm"),
    ])

    # ---- 2) Start SITL and spawn the Iris into that world ------------------
    iris = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ardupilot_gz_bringup, "launch", "robots", "robot.launch.py"
            )
        ),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name"),
            "robot_name": LaunchConfiguration("robot_name"),
            "x": LaunchConfiguration("spawn_x"),
            "y": LaunchConfiguration("spawn_y"),
            "z": LaunchConfiguration("spawn_z"),
            "Y": LaunchConfiguration("spawn_yaw"),
            "sdf_file": iris_sdf_file,
            "command": "arducopter",
            "defaults": sitl_defaults,
        }.items(),
    )

    # ---- 3) Bring up MAVROS so the drone can be flown with commands -------
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pkg_mavros, "launch", "apm.launch")
        ),
        launch_arguments={
            "fcu_url": LaunchConfiguration("fcu_url"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_mavros")),
    )

    return LaunchDescription(
        [
            world_path_arg,
            world_name_arg,
            robot_name_arg,
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            spawn_yaw_arg,
            use_mavros_arg,
            fcu_url_arg,
            use_gui_arg,
            set_resource_path,
            # Applies to every Node launched afterward, including ones
            # pulled in via IncludeLaunchDescription (Python or XML) that
            # we don't control directly -- mavros, robot_state_publisher,
            # ros_gz_bridge all need clock sourced from /clock, not the
            # wall clock, so timestamps line up with sim time.
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            gz_sim_headless,
            iris,
            mavros,
        ]
    )
