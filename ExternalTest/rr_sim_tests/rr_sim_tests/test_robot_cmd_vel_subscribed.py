#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import unittest

import launch
import launch_testing.actions
import launch_testing.markers

import rclpy
from geometry_msgs.msg import Twist

import pytest

from rr_sim_tests.utils.wait_for_spawned_entity import wait_for_spawned_entity
from rr_sim_tests.utils.utils import CmdVelPublisher

"""
Test robot cmd_vel subscription
"""
LAUNCH_ARG_ROBOT_NAMESPACE = 'robot_namespace'
LAUNCH_ARG_ROBOT_NAME =  'robot_name'
LAUNCH_ARG_ROBOT_TWIST_LINEAR = 'twist_linear'
LAUNCH_ARG_ROBOT_TWIST_ANGULAR = 'twist_angular'

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot_namespace = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_NAMESPACE, default='')
    robot_name= launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_NAME, default='')

    pos = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_TWIST_LINEAR, default='2.0, 0.0, 0.0')
    rot = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_TWIST_ANGULAR, default='0.0, 0.0, 1.8')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_NAMESPACE,
            default_value=robot_namespace,
            description='robot namespace'),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_NAME,
            default_value=robot_name,
            description='robot name'),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_TWIST_LINEAR,
            default_value=pos,
            description="Robot twist linear (xyz). Eg:'0.0, 0.0, 0.0'"),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_TWIST_ANGULAR,
            default_value=rot,
            description="Robot twist angular (roll, pitch, yaw). Eg:'0.0, 0.0, 0.0'"),
        launch_testing.actions.ReadyToTest()
    ])

class TestRobotTwist(unittest.TestCase):
    def test_twist_robot(self, proc_output, test_args):
        rclpy.init()

        arghas = lambda arg: arg in test_args
        argstr = lambda arg: str(test_args[arg]) if arg in test_args else ''
        
        # Query robot current pose
        assert(arghas(LAUNCH_ARG_ROBOT_NAME))
        robot_name = argstr(LAUNCH_ARG_ROBOT_NAME)
        is_robot_found, robot_current_pose = wait_for_spawned_entity(robot_name)
        assert is_robot_found, f'Robot named {robot_name} unavailable!'

        # Prepare robot twist data
        robot_twist = Twist()
        assert(arghas(LAUNCH_ARG_ROBOT_TWIST_LINEAR))
        pos = argstr(LAUNCH_ARG_ROBOT_TWIST_LINEAR).split(',')
        robot_twist.linear.x = float(pos[0])
        robot_twist.linear.y = float(pos[1])
        robot_twist.linear.z = float(pos[2])
        
        assert(arghas(LAUNCH_ARG_ROBOT_TWIST_ANGULAR))
        rot = argstr(LAUNCH_ARG_ROBOT_TWIST_ANGULAR).split(',')
        robot_twist.angular.x = float(rot[0])
        robot_twist.angular.y = float(rot[1])
        robot_twist.angular.z = float(rot[2])
        
        # Command the robot to move with twist data
        with CmdVelPublisher(in_robot_namespace=argstr(LAUNCH_ARG_ROBOT_NAMESPACE), in_robot_twist=robot_twist) as cmd_vel_publisher:
            # Check for the robot having been twisted if it does susbcribe to /cmd_vel
            is_twisted = cmd_vel_publisher.wait_for_robot_twisted(robot_name, robot_current_pose, in_timeout=5.0)
            assert is_twisted, f'{robot_name} failed being twisted by cmd_vel'
        rclpy.shutdown()
