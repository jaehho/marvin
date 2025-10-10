#!/usr/bin/env python3
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    ld = LaunchDescription()

    # --- Args ---
    use_sim = LaunchConfiguration('use_sim')
    ld.add_action(DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='Start robot in Gazebo simulation.'
    ))

    # --- Robot Description (URDF/Xacro) ---
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("marvin"),
            "urdf",
            "marvin_robot.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # --- SRDF ---
    robot_description_semantic_path = os.path.join(
        get_package_share_directory("marvin_moveit"),
        "config",
        "marvin.srdf",
    )
    with open(robot_description_semantic_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # --- Kinematics YAML ---
    kinematics_yaml_path = os.path.join(
        get_package_share_directory("marvin_moveit"),
        "config",
        "kinematics.yaml",
    )
    with open(kinematics_yaml_path, "r") as f:
        kinematics_yaml = yaml.safe_load(f)

    # --- Servo YAMLs ---
    left_servo_yaml_path = os.path.join(
        get_package_share_directory("marvin_moveit"),
        "config",
        "moveit_servo_left.yaml",
    )
    with open(left_servo_yaml_path, "r") as f:
        left_servo_params = {"moveit_servo": yaml.safe_load(f)}

    right_servo_yaml_path = os.path.join(
        get_package_share_directory("marvin_moveit"),
        "config",
        "moveit_servo_right.yaml",
    )
    with open(right_servo_yaml_path, "r") as f:
        right_servo_params = {"moveit_servo": yaml.safe_load(f)}

    # --- PlanningSceneMonitor options (private topic) ---
    # psm_params = {
    #     "planning_scene_monitor_options": {
    #         "name": "planning_scene_monitor",
    #         "robot_description": "robot_description",
    #         "joint_state_topic": "/joint_states",
    #         "publish_planning_scene_topic": "~publish_planning_scene", # or /publish_planning_scene
    #         "monitored_planning_scene_topic": "/planning_scene",
    #     }
    # }

    # --- Nodes ---
    left_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        namespace="left",
        parameters=[
            {"use_gazebo": use_sim},
            left_servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            #psm_params,
        ],
        output="screen",
    )

    right_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        namespace="right",
        parameters=[
            {"use_gazebo": use_sim},
            right_servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            #psm_params,
        ],
        output="screen",
    )

    ld.add_action(left_servo_node)
    ld.add_action(right_servo_node)
    return ld
