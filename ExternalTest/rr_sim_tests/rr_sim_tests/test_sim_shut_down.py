#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import time
import unittest

import launch
import launch_testing.actions
import launch_testing.markers

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ue_msgs.action import SimShutDown

from rr_sim_tests.utils.shut_down_sim import SimShutDownActionClient
import pytest

"""
Test UE sim shutdown
"""

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch_testing.actions.ReadyToTest()
    ])

class TestSimShutDown(unittest.TestCase):
    def test_shut_down_sim(self, proc_output):
        rclpy.init()
        with SimShutDownActionClient() as sim_shut_down_client:
            sim_shut_down_client.shut_down_sim()
        rclpy.shutdown()
