#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import unittest
import logging

import launch
import launch_testing.actions
import launch_testing.markers
import launch_ros.actions
from rosgraph_msgs.msg import Clock

from rr_sim_tests.utils.wait_for_topics import WaitForTopics
import pytest

"""
Test if clock is being published to
"""
TOPIC_NAME_CLOCK = 'clock'

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch_testing.actions.ReadyToTest()
    ])

class TestClockPublication(unittest.TestCase):
    def test_check_if_clock_published(self, proc_output):
        with WaitForTopics([(TOPIC_NAME_CLOCK, Clock)], in_timeout=5.0):
            logging.info('Clock being published!')
