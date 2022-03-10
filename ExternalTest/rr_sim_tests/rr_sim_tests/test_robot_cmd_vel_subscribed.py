#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import asyncio
import time
import unittest

import launch
import launch_testing.actions
import launch_testing.markers

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist

from rr_sim_tests.utils.wait_for_spawned_robot import wait_for_spawned_robot
import pytest

TOPIC_NAME_CMD_VEL = 'cmd_vel'
PUBLISHING_NUM = 1

class CmdVelPublisher(Node):
    def __init__(self, in_robot_namespace:str, in_robot_twist:Twist):
        super().__init__('cmd_vel_publisher')
        self._cmd_vel_topic = f'/{in_robot_namespace}/{TOPIC_NAME_CMD_VEL}'\
                                if ((in_robot_namespace != None) and len(in_robot_namespace.strip()) > 0)\
                                else f'/{TOPIC_NAME_CMD_VEL}'
        self._twist_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._twist = in_robot_twist
        
        self._twist_pub_count = 0
        self._is_pub_finished = False

        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(self)
        self._timer = self.create_timer(0.1, self.twist_robot)

        while rclpy.ok() and not self._is_pub_finished:
            self._executor.spin_once(timeout_sec = 0)
        print(f'Finished publishing Twist to {self._cmd_vel_topic}')

    def __enter__(self):
        return self

    def __exit__(self, exep_type, exep_value, trace):
        if exep_type is not None:
            raise Exception('Exception occured, value: ', exep_value)
        self._executor.shutdown(timeout_sec = 0.5)
        self.destroy_node()

    @property
    def cmd_vel_topic(self):
        return self._cmd_vel_topic

    async def twist_robot(self):
        if (self._twist_pub_count < PUBLISHING_NUM):
            self.get_logger().info(f'Publishing to {self._cmd_vel_topic}: {self._twist}')
            self._twist_pub.publish(self._twist)
            self._twist_pub_count += 1
        else:
            self._is_pub_finished = True
            self._timer.cancel()
        await asyncio.sleep(0)

    def wait_for_robot_twisted(self, in_robot_name, in_robot_prev_pose, in_timeout=5.0):
        assert self._is_pub_finished
        is_robot_found, robot_pose = wait_for_spawned_robot(in_robot_name, in_timeout)
        assert is_robot_found, f'wait_for_robot_twisted(): {in_robot_name} unavailable!'
        return (robot_pose != in_robot_prev_pose)

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
        is_robot_found, robot_current_pose = wait_for_spawned_robot(robot_name)
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
        with CmdVelPublisher(argstr(LAUNCH_ARG_ROBOT_NAMESPACE), robot_twist) as cmd_vel_publisher:
            # Check for the robot having been twisted if it does susbcribe to /cmd_vel
            assert cmd_vel_publisher.wait_for_robot_twisted(robot_name, robot_current_pose, in_timeout=5.0)
        rclpy.shutdown()
