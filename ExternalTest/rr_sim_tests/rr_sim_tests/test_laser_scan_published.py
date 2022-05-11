#! /usr/bin/env python
import unittest

import launch
import launch_testing.actions
import launch_testing.markers
import launch_ros.actions
from sensor_msgs.msg import LaserScan

from rr_sim_tests.utils.wait_for_topics import WaitForTopics
import pytest

"""
Test if laser scan by lidar is being published
"""
LAUNCH_ARG_SCAN_TOPICS = 'scan_topics'

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    scan_topics = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_SCAN_TOPICS, default='scan')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_SCAN_TOPICS,
            default_value=scan_topics,
            description='topics of laser scan'),
        launch_testing.actions.ReadyToTest()
    ])

class TestLaserScanPublication(unittest.TestCase):
    def test_check_if_laser_scan_published(self, proc_output, test_args):
        with WaitForTopics([(str(scan_topic).strip(), LaserScan) for scan_topic in test_args[LAUNCH_ARG_SCAN_TOPICS].split(',')], in_timeout=5.0):
            print('Laser scan is being published!')