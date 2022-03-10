#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import time
import unittest

import launch
import launch_testing.actions
import launch_testing.markers

import rclpy
from rclpy.node import Node

from ue_msgs.srv import DeleteEntity

from rr_sim_tests.utils.wait_for_service import wait_for_service
from rr_sim_tests.utils.wait_for_spawned_robot import wait_for_spawned_robot

import pytest

"""
Test robot removal
"""
LAUNCH_ARG_ROBOT_NAME = 'robot_name'
SERVICE_NAME_DELETE_ENTITY = 'DeleteEntity'

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # For [robot_name] arg
    robot_name = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_NAME, default='')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_NAME,
            default_value=robot_name,
            description='Robot unique name'),
        launch_testing.actions.ReadyToTest()
    ])

def remove_robot(in_robot_name, in_timeout=5.0):
    assert(len(in_robot_name) > 0)
    node = rclpy.create_node(f'spawn_{in_robot_name}')
    cli = wait_for_service(node, DeleteEntity, SERVICE_NAME_DELETE_ENTITY)
    if not cli.service_is_ready():
        return False

    # Prepare DeleteEntity request
    req = DeleteEntity.Request()
    req.name = in_robot_name

    # Async invoke DeleteEntity service
    future = cli.call_async(req)
    try:
        start = time.time()
        while (time.time() - start < in_timeout):
            rclpy.spin_once(node)
            if future.done():
                break
    finally:
        node.destroy_node()
    return True

class TestRobotRemove(unittest.TestCase):
    def test_remove_robot(self, proc_output, test_args):
        rclpy.init()

        arghas = lambda arg: arg in test_args
        argstr = lambda arg: str(test_args[arg]) if arg in test_args else ''
        
        assert(arghas(LAUNCH_ARG_ROBOT_NAME))
        robot_name = argstr(LAUNCH_ARG_ROBOT_NAME)

        is_robot_found, _ = wait_for_spawned_robot(robot_name, 1.0)
        assert is_robot_found, f'{robot_name} is not available!'

        assert remove_robot(robot_name)
        is_robot_found, _ = wait_for_spawned_robot(robot_name, 1.0)
        assert False == is_robot_found, f'{robot_name} failed being removed!'
        rclpy.shutdown()
