#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import asyncio
import time

# rclpy
import rclpy
from rclpy.node import Node

# rr_sim_tests
from rr_sim_tests.utils.wait_for_service import wait_for_service
from rr_sim_tests.utils.wait_for_spawned_entity import wait_for_spawned_entity

# UE_msgs
from ue_msgs.msg import EntityState
from ue_msgs.srv import SpawnEntity

# other ros
from geometry_msgs.msg import Pose

SERVICE_NAME_SPAWN_ENTITY = "SpawnEntity"

def wait_for_entity_moving(in_entity_name : str,
                           in_entity_prev_pose : Pose,
                           in_watch_rate=10.0, #Hz
                           in_timeout=5.0):
    entity_prev_pos = in_entity_prev_pose.position
    start_time = time.time()
    wait_interval = 1.0/in_watch_rate
    while rclpy.ok() and (time.time() - start_time) < in_timeout:
        time.sleep(wait_interval)
        is_entity_found, entity_pose = wait_for_spawned_entity(
            in_entity_name, in_timeout=5.0
        )
        assert (
            is_entity_found
        ), f"wait_for_entity_moving(): {in_entity_name} unavailable!"

        # Disregarding entity's Z change as probably just out of falling upon gravity
        entity_new_pos = entity_pose.position
        if entity_new_pos.x != entity_prev_pos.x or \
            entity_new_pos.y != entity_prev_pos.y or \
            entity_pose.orientation != in_entity_prev_pose.orientation:
            return True
    return False

def spawn_entity(
    in_entity_model,
    in_entity_name,
    in_entity_namespace,
    in_entity_ref_frame,
    in_entity_pose,
    in_service_namespace="",
    in_entity_tags=None,
    in_entity_json="",
    in_timeout=5.0,
):
    assert len(in_entity_model) > 0
    assert len(in_entity_name) > 0
    node = rclpy.create_node(f"spawn_{in_entity_name}")
    cli = wait_for_service(
        node, SpawnEntity, in_service_namespace + "/" + SERVICE_NAME_SPAWN_ENTITY
    )
    if not cli.service_is_ready():
        return False

    # Prepare SpawnEntity request
    req = SpawnEntity.Request()
    req.xml = in_entity_model
    req.robot_namespace = in_entity_namespace
    req.state = EntityState()
    req.state.name = in_entity_name
    req.state.reference_frame = in_entity_ref_frame
    req.state.pose = in_entity_pose
    req.tags = in_entity_tags
    req.json_parameters = in_entity_json

    # Async invoke SpawnEntity service
    future = cli.call_async(req)
    try:
        rclpy.spin_until_future_complete(node, future, timeout_sec=in_timeout)
    finally:
        node.destroy_node()
    return True
