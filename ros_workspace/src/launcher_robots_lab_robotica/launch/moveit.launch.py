import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur3e", package_name="launcher_robots_lab_robotica")
        .robot_description_semantic(Path("config") / "ur3e_rg2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
    }

    # Espera a que robot_state_publisher publique el topic robot_description
    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": True,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("launcher_robots_lab_robotica"), "config", "moveit_ur.rviz"]
            ),
        ],
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true", description="¿Lanzar RViz?"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            wait_robot_description,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_robot_description,
                    on_exit=[move_group_node, rviz_node],
                )
            ),
        ]
    )
