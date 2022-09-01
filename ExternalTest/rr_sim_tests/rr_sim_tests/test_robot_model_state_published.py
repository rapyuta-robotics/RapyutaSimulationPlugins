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

from rr_sim_tests.utils.wait_for_topics import WaitForTopics

class ModelStateSubscriber(Node):
    def __init__(self, in_robot_namespace:str):
        super().__init__('model_state_subscriber')
        self._is_model_state_received = False
        self._model_state_topic = f'/{in_robot_namespace}/ue_ros/model_state' if ((in_robot_namespace != None) and len(in_robot_namespace.strip()) > 0) else '/ue_ros/model_state'
        self.create_subscription(
            EntityState,
            self._model_state_topic,
            self.on_model_state_published,
            10)

    @property
    def model_state_topic(self):
        return self._model_state_topic

    @property
    def is_model_state_received(self):
        return self._is_model_state_received

    def on_model_state_published(self, msg):
        self.get_logger().info(
            f'EntityState published to {self._model_state_topic}:\n'
            f'- name:{msg.name}\n'
            f'- reference_frame:{msg.reference_frame}\n'
            f'- pose: {msg.pose}\n'
            f'- twist: {msg.twist}'
        )
        self._is_model_state_received = True

    def wait_for_model_state_data(self, in_timeout = 5.0):
        start_time = time.time()
        while (not self._is_model_state_received) and ((time.time() - start_time) < in_timeout):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._is_model_state_received

"""
Test if model state is being published for a given robot with [robot_namespace]
"""
LAUNCH_ARG_ROBOT_NAMESPACE = 'robot_namespace'

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot_namespace = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_NAMESPACE, default='')
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_NAMESPACE,
            default_value=robot_namespace,
            description='Robot namespace being used as the namespace of the model state topic'),
        launch_testing.actions.ReadyToTest()
    ])

class TestRobotModelStateFetch(unittest.TestCase):
    def test_fetch_robot_model_state(self, proc_output, test_args):
        rclpy.init()
        model_state_subscriber = ModelStateSubscriber(test_args[LAUNCH_ARG_ROBOT_NAMESPACE] if LAUNCH_ARG_ROBOT_NAMESPACE in test_args else '')

        # Wait for model state data
        with WaitForTopics([(model_state_subscriber.model_state_topic, EntityState)], in_timeout=5.0):
            assert model_state_subscriber.wait_for_model_state_data(in_timeout=5.0)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        model_state_subscriber.destroy_node()
        rclpy.shutdown()
