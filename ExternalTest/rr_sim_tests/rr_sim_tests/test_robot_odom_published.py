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
from rr_sim_tests.utils.base_subscriber import BaseSubscriber, SUBSCRIBER_DEFAULT_QOS
from distutils.util import strtobool

TOPIC_NAME_ODOM = 'odom'
ODOM_SUBSCRIBER_RATE = 10.0 #Hz

# TF topics are always in global namespace
# https://github.com/ros2/geometry2/blob/iron/tf2_ros/include/tf2_ros/transform_broadcaster.h#L96
# https://github.com/ros2/geometry2/blob/iron/tf2_ros/include/tf2_ros/static_transform_broadcaster.h#L112
TOPIC_NAME_TF = '/tf'
TOPIC_NAME_TF_STATIC = '/tf_static'
TF_SUBSCRIBER_RATE = 10.0 #Hz

class TFSubscriber(BaseSubscriber):
    def __init__(self, in_robot_namespace:str,
                 in_tf_static:bool):
        super().__init__(in_robot_namespace, in_topic_name=(TOPIC_NAME_TF_STATIC if in_tf_static else TOPIC_NAME_TF) if in_tf_published else None,
                         in_subscriber_rate=TF_SUBSCRIBER_RATE)

        self._subscriber = self.create_subscription(
            TFMessage,
            self._full_topic_name,
            self.on_tf_published,
            SUBSCRIBER_DEFAULT_QOS)

    def on_tf_published(self, msg):
        self.get_logger().info(
            f'Transforms published to {self._full_topic_name}:\n'
            f'- transforms:{msg.transforms}\n'
        )
        self._topic_data_arrived = True

class OdomSubscriber(BaseSubscriber):
    def __init__(self, in_robot_namespace:str,
                 in_topic_name: str=TOPIC_NAME_ODOM,
                 in_tf_published:bool=True, in_tf_static:bool=False):
        super().__init__(in_robot_namespace, in_topic_name, in_subscriber_rate=ODOM_SUBSCRIBER_RATE)
        self._subscriber = self.create_subscription(
            Odometry,
            self._full_topic_name,
            self.on_odom_published,
            SUBSCRIBER_DEFAULT_QOS)

        self._tfSubscriber = TFSubscriber(in_robot_namespace, in_tf_static) if in_tf_published else None

    def on_odom_published(self, msg):
        self.get_logger().info(
            f'Odometry published to {self._full_topic_name}:\n'
            f'- timestamp:{msg.header.stamp}\n'
            f'- frame id:{msg.header.frame_id}\n'
            f'- child frame id: {msg.child_frame_id}\n'
            f'- pose: {msg.pose}\n'
            f'- twist: {msg.twist}'
        )
        self._topic_data_arrived = True

    @property  
    def tf_topic(self):
        return self._tfSubscriber.full_topic if self._tfSubscriber else ''

    def wait_for_tf_data(self, in_timeout = 5.0):
        if self._tfSubscriber:
            return self._tfSubscriber.wait_for_topic_data(in_timeout)
        else:
            assert False, "No tfSubscriber created"

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
                                         in_tf_published=is_tf_published,
                                         in_tf_static=bool(strtobool(test_args[LAUNCH_ARG_TF_STATIC])) if LAUNCH_ARG_TF_STATIC in test_args else False)

        # Wait for odom data
        with WaitForTopics([(odom_subscriber.full_topic, Odometry)], in_timeout=5.0):
            assert odom_subscriber.wait_for_topic_data(in_timeout=5.0)

        # Wait for tf data
        if is_tf_published:
            with WaitForTopics([(odom_subscriber.tf_topic, TFMessage)], in_timeout=5.0):
                assert odom_subscriber.wait_for_tf_data(in_timeout=5.0)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        odom_subscriber.destroy_node()
        rclpy.shutdown()
