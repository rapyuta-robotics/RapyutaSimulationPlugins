#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import os
import time
import rclpy

from ue_msgs.srv import GetEntityState
from rr_sim_tests.utils.wait_for_service import wait_for_service

SERVICE_NAME_GET_ENTITY_STATE = 'GetEntityState'

def wait_for_spawned_entity(in_entity_name, service_namespace='', in_timeout=10.0):
    print(f'Waiting for entity [{in_entity_name}]...')
    node = rclpy.create_node(f'wait_for_{in_entity_name}')
    cli = wait_for_service(node,GetEntityState,  os.path.join(service_namespace, SERVICE_NAME_GET_ENTITY_STATE), in_timeout)
    if not cli.service_is_ready():
        return False

    # Query entity state
    is_spawned = False
    start = time.time()

    req = GetEntityState.Request()
    req.name = in_entity_name
    future = cli.call_async(req)
    entity_pose = None
    try:
        while (time.time() - start < in_timeout):
            rclpy.spin_once(node)
            if future.done():
                result = future.result()
                node.get_logger().info(
                    f'Result of [GetEntityState] for {req.name}:\n'
                    f'- success:{result.success}\n'
                    f'- name: {result.state.name}\n'
                    f'- pose: {result.state.pose}\n'
                    f'- reference frame: {result.state.reference_frame}'
                )
                entity_pose = result.state.pose
                assert(result.state.name == req.name)
                is_spawned = result.success
                break
            time.sleep(0.5)
    finally:
        node.destroy_node()
    return is_spawned, entity_pose
