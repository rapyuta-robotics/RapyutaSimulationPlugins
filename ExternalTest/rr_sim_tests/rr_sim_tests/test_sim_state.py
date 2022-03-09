#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import time
import unittest

import launch
import launch_testing.actions
import launch_testing.markers

from ue_msgs.srv import SpawnEntity, DeleteEntity, GetEntityState, SetEntityState, Attach

import rclpy
from rclpy.node import Node

from rr_sim_tests.utils.wait_for_service import wait_for_service

import pytest

"""
Test if UE simulation state service servers exist
"""
LAUNCH_ARG_TIMEOUT = 'timeout'

SERVICE_NAME_SPAWN_ENTITY = 'SpawnEntity'
SERVICE_NAME_DELETE_ENTITY = 'DeleteEntity'
SERVICE_NAME_GET_ENTITY_STATE = 'GetEntityState'
SERVICE_NAME_SET_ENTITY_STATE = 'SetEntityState'
SERVICE_NAME_ATTACH_ENTITY = 'Attach'

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    timeout = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_TIMEOUT, default='5')
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_TIMEOUT,
            default_value=timeout,
            description='Sim state initialization timeout in sec'),
        launch_testing.actions.ReadyToTest()
    ])

def wait_for_sim_state(in_timeout=10.0):
    node = rclpy.create_node(f'wait_for_sim_state_service_servers')
    
    cli = wait_for_service(node, SpawnEntity, SERVICE_NAME_SPAWN_ENTITY, in_timeout)
    if not cli.service_is_ready():
        node.destroy_node()
        return False

    cli = wait_for_service(node, DeleteEntity, SERVICE_NAME_DELETE_ENTITY, in_timeout)
    if not cli.service_is_ready():
        node.destroy_node()
        return False

    cli = wait_for_service(node, GetEntityState, SERVICE_NAME_GET_ENTITY_STATE, in_timeout)
    if not cli.service_is_ready():
        node.destroy_node()
        return False

    cli = wait_for_service(node, SetEntityState, SERVICE_NAME_SET_ENTITY_STATE, in_timeout)
    if not cli.service_is_ready():
        node.destroy_node()
        return False

    cli = wait_for_service(node, Attach, SERVICE_NAME_ATTACH_ENTITY, in_timeout)
    if not cli.service_is_ready():
        node.destroy_node()
        return False

    node.destroy_node()
    return True

class TestSimState(unittest.TestCase):
    def test_check_sim_state_ready(self, proc_output, test_args):
        rclpy.init()
        assert(LAUNCH_ARG_TIMEOUT in test_args)
        assert wait_for_sim_state(float(test_args[LAUNCH_ARG_TIMEOUT]))
        rclpy.shutdown()
