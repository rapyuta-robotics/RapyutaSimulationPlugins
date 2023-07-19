#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import time
import unittest

import launch
import launch_testing.actions
import launch_testing.markers
from ue_msgs.msg import EntityState
import pytest

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy, QoSProfile

from rr_sim_tests.utils.wait_for_topics import WaitForTopics
from rr_sim_tests.utils.base_subscriber import BaseSubscriber

TOPIC_NAME_MODEL_STATE = "ue_ros/model_state"
MODEL_STATE_SUBSCRIBER_RATE = 10.0 #Hz

class ModelStateSubscriber(BaseSubscriber):
    def __init__(self, in_robot_namespace:str,
                 in_topic_name: str=TOPIC_NAME_MODEL_STATE):
        super().__init__(in_robot_namespace, in_topic_name, MODEL_STATE_SUBSCRIBER_RATE)
        self._subscriber = self.create_subscription(
            EntityState,
            self._full_topic_name,
            self.on_model_state_published,
            10)

    def on_model_state_published(self, msg):
        self.get_logger().info(
            f'EntityState published to {self._full_topic_name}:\n'
            f'- name:{msg.name}\n'
            f'- reference_frame:{msg.reference_frame}\n'
            f'- pose: {msg.pose}\n'
            f'- twist: {msg.twist}'
        )
        self._topic_data_arrived = True

"""
Test if model state is being published for a given robot with [robot_namespace]
"""
LAUNCH_ARG_ENTITY_NAMESPACE = 'robot_namespace'

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot_namespace = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ENTITY_NAMESPACE, default='')
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ENTITY_NAMESPACE,
            default_value=robot_namespace,
            description='Robot namespace being used as the namespace of the model state topic'),
        launch_testing.actions.ReadyToTest()
    ])

class TestRobotModelStateFetch(unittest.TestCase):
    def test_fetch_robot_model_state(self, proc_output, test_args):
        rclpy.init()
        model_state_subscriber = ModelStateSubscriber(test_args[LAUNCH_ARG_ENTITY_NAMESPACE] if LAUNCH_ARG_ENTITY_NAMESPACE in test_args else '')

        # Wait for model state data
        with WaitForTopics([(model_state_subscriber.full_topic, EntityState)], in_timeout=5.0):
            assert model_state_subscriber.wait_for_topic_data(in_timeout=5.0)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        model_state_subscriber.destroy_node()
        rclpy.shutdown()
