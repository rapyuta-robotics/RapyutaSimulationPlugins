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
from rr_sim_tests.utils.wait_for_spawned_entity import wait_for_spawned_entity

import pytest

"""
Test entity removal
"""
LAUNCH_ARG_ENTITY_NAME = 'entity_name'
SERVICE_NAME_DELETE_ENTITY = 'DeleteEntity'

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # For [entity_name] arg
    entity_name = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ENTITY_NAME, default='')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ENTITY_NAME,
            default_value=entity_name,
            description='Robot unique name'),
        launch_testing.actions.ReadyToTest()
    ])

def remove_entity(in_entity_name, in_timeout=5.0):
    assert(len(in_entity_name) > 0)
    node = rclpy.create_node(f'spawn_{in_entity_name}')
    cli = wait_for_service(node, DeleteEntity, SERVICE_NAME_DELETE_ENTITY)
    if not cli.service_is_ready():
        return False

    # Prepare DeleteEntity request
    req = DeleteEntity.Request()
    req.name = in_entity_name

    # Async invoke DeleteEntity service
    future = cli.call_async(req)
    try:
        rclpy.spin_until_future_complete(node, future, timeout_sec=in_timeout)
    finally:
        node.destroy_node()
    return True

class TestRobotRemove(unittest.TestCase):
    def test_remove_entity(self, proc_output, test_args):
        rclpy.init()

        arghas = lambda arg: arg in test_args
        argstr = lambda arg: str(test_args[arg]) if arg in test_args else ''
        
        assert(arghas(LAUNCH_ARG_ENTITY_NAME))
        entity_name = argstr(LAUNCH_ARG_ENTITY_NAME)

        is_entity_found, _ = wait_for_spawned_entity(entity_name, 5.0)
        assert is_entity_found, f'{entity_name} is not available!'

        assert remove_entity(entity_name)
        is_entity_found, _ = wait_for_spawned_entity(entity_name, 5.0)
        assert False == is_entity_found, f'{entity_name} failed being removed!'
        rclpy.shutdown()
