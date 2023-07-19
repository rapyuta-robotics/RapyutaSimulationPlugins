#! /usr/bin/env python3
# Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

import unittest

import launch
import launch_testing.actions
import launch_testing.markers

import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped

import pytest

from rr_sim_tests.utils.wait_for_spawned_entity import wait_for_spawned_entity
from rr_sim_tests.utils.path_publisher import PathPublisher

"""
Test robot path subscription
"""
LAUNCH_ARG_ENTITY_NAMESPACE = 'robot_namespace'
LAUNCH_ARG_ENTITY_NAME = 'robot_name'
LAUNCH_ARG_ROBOT_PATH = 'robot_path'

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot_namespace = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ENTITY_NAMESPACE, default='')
    robot_name= launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ENTITY_NAME, default='')

    robot_path = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_PATH, default='0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ENTITY_NAMESPACE,
            default_value=robot_namespace,
            description='robot namespace'),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ENTITY_NAME,
            default_value=robot_name,
            description='robot name'),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_PATH,
            default_value=robot_path,
            description="robot path (lx,ly,lz, rx,ry,rz,rw). Eg:'0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0'"),
        launch_testing.actions.ReadyToTest()
    ])

class TestRobotFollowPath(unittest.TestCase):
    def test_robot_follow_path(self, proc_output, test_args):
        rclpy.init()

        arghas = lambda arg: arg in test_args
        argstr = lambda arg: str(test_args[arg]) if arg in test_args else ''
        
        # Query robot current pose
        assert(arghas(LAUNCH_ARG_ENTITY_NAME))
        robot_name = argstr(LAUNCH_ARG_ENTITY_NAME)
        is_robot_found, _ = wait_for_spawned_entity(robot_name)
        assert is_robot_found, f'Robot named {robot_name} unavailable!'

        # Prepare robot path data
        print(robot_name)
        node = rclpy.create_node(f'{robot_name}_path')
        robot_path = Path()
        robot_path.header.frame_id = "map_origin"
        robot_path.header.stamp = node.get_clock().now().to_msg()
        assert(arghas(LAUNCH_ARG_ROBOT_PATH))
        path_poses = argstr(LAUNCH_ARG_ROBOT_PATH).split(',')

        poses_num = int(len(path_poses) / 7)
        print(f'path_poses: {path_poses}')
        print(f'target poses num: {poses_num}')
        for i in range(poses_num):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = float(path_poses[0*i])
            pose_stamped.pose.position.y = float(path_poses[0*i+1])
            pose_stamped.pose.position.z = float(path_poses[0*i+2])
            pose_stamped.pose.orientation.x = float(path_poses[0*i+3])
            pose_stamped.pose.orientation.y = float(path_poses[0*i+4])
            pose_stamped.pose.orientation.z = float(path_poses[0*i+5])
            pose_stamped.pose.orientation.w = float(path_poses[0*i+6])
            robot_path.poses.append(pose_stamped)
        
        # Command the robot to move following path
        with PathPublisher(in_robot_namespace=argstr(LAUNCH_ARG_ENTITY_NAMESPACE), in_topic_name='ue_ros/robot_path', in_robot_path=robot_path) as path_publisher:
            # Check for the robot having followed the patth if it does susbcribe to ue_ros/robot_path
            is_path_followed = path_publisher.wait_for_robot_waypoint_reached(robot_name, robot_path.poses[0].pose, in_timeout=20.0)
            assert is_path_followed, f'{robot_name} failed following path published to {path_publisher.path_topic}'
        rclpy.shutdown()
