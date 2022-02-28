#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import time
import unittest

import launch
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import pytest

import rclpy
from rclpy.node import Node

from rr_sim_tests.utils.wait_for_topics import WaitForTopics
from distutils.util import strtobool

class OdomSubscriber(Node):
    def __init__(self, in_robot_namespace:str, in_tf_published:bool, in_tf_static:bool):
        super().__init__('odom_subscriber')
        self._is_odom_received = False
        self._odom_topic = f'/{in_robot_namespace}/odom' if ((in_robot_namespace != None) and len(in_robot_namespace.strip()) > 0) else '/odom'
        self.create_subscription(
            Odometry,
            self._odom_topic,
            self.on_odom_published,
            10)

        self._tf_topic = ('/tf_static' if in_tf_static else '/tf') if in_tf_published else None
        self._is_tf_received = False
        if in_tf_published:
            self.create_subscription(
                TFMessage,
                self._tf_topic,
                self.on_tf_published,
                10)

    @property
    def odom_topic(self):
        return self._odom_topic

    @property
    def is_odom_received(self):
        return self._is_odom_received
      
    def on_odom_published(self, msg):
        self.get_logger().info(
            f'Odometry published to {self._odom_topic}:\n'
            f'- timestamp:{msg.header.stamp}\n'
            f'- frame id:{msg.header.frame_id}\n'
            f'- child frame id: {msg.child_frame_id}\n'
            f'- pose: {msg.pose}\n'
            f'- twist: {msg.twist}'
        )
        self._is_odom_received = True

    @property
    def tf_topic(self):
        return self._tf_topic

    def on_tf_published(self, msg):
        self.get_logger().info(
            f'Transforms published to {self._tf_topic}:\n'
            f'- transforms:{msg.transforms}\n'
        )
        self._is_tf_received = True

    def wait_for_odom_data(self, in_timeout = 5.0):
        start_time = time.time()
        while (not self._is_odom_received) and ((time.time() - start_time) < in_timeout):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._is_odom_received
        
    def wait_for_tf_data(self, in_timeout = 5.0):
        start_time = time.time()
        while (not self._is_tf_received) and ((time.time() - start_time) < in_timeout):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._is_tf_received
"""
Test if odom is being published for a given robot with [robot_namespace]
"""
LAUNCH_ARG_ROBOT_NAMESPACE = 'robot_namespace'
LAUNCH_ARG_TF_PUBLISHED = 'is_tf_published'
LAUNCH_ARG_TF_STATIC = 'is_tf_static'

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot_namespace = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_NAMESPACE, default='')
    is_tf_published = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_TF_PUBLISHED, default='False')
    is_tf_static = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_TF_STATIC, default='False')
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_NAMESPACE,
            default_value=robot_namespace,
            description='Robot namespace being used as the namespace of the odom topic'),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_TF_PUBLISHED,
            default_value=is_tf_published,
            description='Whether tf is published'),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_TF_STATIC,
            default_value=is_tf_static,
            description='Whether tf is static'),
        launch_testing.actions.ReadyToTest()
    ])

class TestRobotOdomFetch(unittest.TestCase):
    def test_fetch_robot_odom(self, proc_output, test_args):
        rclpy.init()
        is_tf_published = bool(strtobool(test_args[LAUNCH_ARG_TF_PUBLISHED])) if LAUNCH_ARG_TF_PUBLISHED in test_args else False
        odom_subscriber = OdomSubscriber(test_args[LAUNCH_ARG_ROBOT_NAMESPACE] if LAUNCH_ARG_ROBOT_NAMESPACE in test_args else '',
                                         is_tf_published,
                                         bool(strtobool(test_args[LAUNCH_ARG_TF_STATIC])) if LAUNCH_ARG_TF_STATIC in test_args else False)

        # Wait for odom data
        with WaitForTopics([(odom_subscriber.odom_topic, Odometry)], in_timeout=5.0):
            assert odom_subscriber.wait_for_odom_data(in_timeout=5.0)

        # Wait for tf data
        if is_tf_published:
            with WaitForTopics([(odom_subscriber.tf_topic, TFMessage)], in_timeout=5.0):
                assert odom_subscriber.wait_for_tf_data(in_timeout=5.0)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        odom_subscriber.destroy_node()
        rclpy.shutdown()
