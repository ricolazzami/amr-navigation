"""Full AMR demo: Gazebo + EKF + SLAM + A* + Nav2 controller + avoidance.

    ros2 launch amr_bringup demo.launch.py
    ros2 launch amr_bringup demo.launch.py headless:=false rviz:=true
    ros2 launch amr_bringup demo.launch.py dynamic:=false
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory("amr_bringup")
    description = get_package_share_directory("amr_description")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    slam_toolbox = get_package_share_directory("slam_toolbox")

    xacro_file = os.path.join(description, "urdf", "amr.urdf.xacro")
    world_file = os.path.join(bringup, "worlds", "maze.sdf")
    rviz_file = os.path.join(bringup, "rviz", "amr.rviz")

    ekf_params = os.path.join(bringup, "config", "ekf.yaml")
    slam_params = os.path.join(bringup, "config", "slam_toolbox.yaml")
    amcl_params = os.path.join(bringup, "config", "amcl.yaml")
    map_yaml = os.path.join(bringup, "maps", "maze.yaml")
    nav2_params = os.path.join(bringup, "config", "nav2_params.yaml")
    planning_params = os.path.join(bringup, "config", "planning.yaml")
    avoidance_params = os.path.join(bringup, "config", "avoidance.yaml")
    mover_params = os.path.join(bringup, "config", "obstacle_mover.yaml")

    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    dynamic = LaunchConfiguration("dynamic")
    ground_truth = LaunchConfiguration("ground_truth")
    world = LaunchConfiguration("world")
    slam_flag = LaunchConfiguration("slam")
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    # ---------------------------------------------------------------- Gazebo
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r -s ", world]}.items(),
        condition=IfCondition(headless),
    )
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
        condition=UnlessCondition(headless),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str),
                "use_sim_time": sim_time,
            }
        ],
    )

    # gz spawn must wait for the server to come up
    spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic", "robot_description",
                    "-name", "amr",
                    "-x", "-4.0",
                    "-y", "-4.0",
                    "-z", "0.02",
                ],
            )
        ],
    )

    bridge_args = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/wheel/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
    ]
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=bridge_args,
        parameters=[{"use_sim_time": sim_time}],
    )

    ground_truth_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        condition=IfCondition(ground_truth),
        arguments=[
            "/world/maze/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        parameters=[{"use_sim_time": sim_time}],
    )

    # ------------------------------------------------------- state estimation
    # gz scopes odometry frames with the model name (amr/odom); robot_localization
    # needs plain odom/base_footprint, so relay them before the filter sees them.
    odom_relay = Node(
        package="amr_bringup",
        executable="odom_frame_relay",
        name="odom_frame_relay",
        output="screen",
        parameters=[{"use_sim_time": sim_time}],
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params, {"use_sim_time": sim_time}],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": slam_params,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(slam_flag),
    )

    # Localisation against the pre-built map. Offline SLAM keeps the pose and
    # the planner's occupancy grid consistent; online SLAM over a box maze
    # drifts relative to the (accurate) EKF odometry and the A* start/obstacle
    # cells land in the wrong place. slam:=true selects the online mapping mode.
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": sim_time, "yaml_filename": map_yaml},
        ],
        condition=UnlessCondition(slam_flag),
    )
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_params, {"use_sim_time": sim_time}],
        condition=UnlessCondition(slam_flag),
    )
    lifecycle_localization = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                condition=UnlessCondition(slam_flag),
                parameters=[
                    {
                        "use_sim_time": sim_time,
                        "autostart": True,
                        "node_names": ["map_server", "amcl"],
                    }
                ],
            )
        ],
    )

    # ------------------------------------------------ Nav2 controller only
    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": sim_time}],
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )
    lifecycle = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": sim_time,
                        "autostart": True,
                        "node_names": ["controller_server"],
                    }
                ],
            )
        ],
    )

    # ------------------------------------------------------ our deliverables
    planner = Node(
        package="amr_planning",
        executable="a_star_node",
        name="a_star_planner",
        output="screen",
        parameters=[planning_params, {"use_sim_time": sim_time}],
    )
    avoidance = Node(
        package="amr_control",
        executable="avoidance_node",
        name="avoidance_node",
        output="screen",
        parameters=[avoidance_params, {"use_sim_time": sim_time}],
    )
    mover = Node(
        package="amr_bringup",
        executable="obstacle_mover",
        name="obstacle_mover",
        output="screen",
        condition=IfCondition(dynamic),
        parameters=[mover_params, {"use_sim_time": sim_time}],
    )

    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                condition=IfCondition(rviz),
                arguments=["-d", rviz_file],
                parameters=[{"use_sim_time": sim_time}],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use Gazebo clock"
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Run Gazebo server only (no GUI). Recommended on 7 GB RAM.",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Start RViz2"
            ),
            DeclareLaunchArgument(
                "dynamic",
                default_value="true",
                description="Drive the moving obstacle",
            ),
            DeclareLaunchArgument(
                "ground_truth",
                default_value="true",
                description="Bridge Gazebo poses for metrics",
            ),
            DeclareLaunchArgument(
                "world", default_value=world_file, description="Gazebo world file"
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="false",
                description=(
                    "true: online SLAM (slam_toolbox). false: localise against "
                    "the pre-built maps/maze map with map_server + AMCL."
                ),
            ),
            gz_headless,
            gz_gui,
            robot_state_publisher,
            bridge,
            ground_truth_bridge,
            spawn,
            odom_relay,
            ekf,
            slam,
            map_server,
            amcl,
            controller,
            lifecycle,
            lifecycle_localization,
            planner,
            avoidance,
            mover,
            rviz_node,
        ]
    )
