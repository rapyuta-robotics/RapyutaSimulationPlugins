#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import asyncio
import time

# rclpy
import rclpy

# other ros
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

# rr_sim_tests
from rr_sim_tests.utils.wait_for_service import wait_for_service
from rr_sim_tests.utils.wait_for_spawned_entity import wait_for_spawned_entity

# UE_msgs
from ue_msgs.msg import EntityState
from ue_msgs.srv import SpawnEntity

SERVICE_NAME_SPAWN_ENTITY = "SpawnEntity"


def spawn_robot(
    in_robot_model,
    in_robot_name,
    in_robot_namespace,
    in_robot_ref_frame,
    in_robot_pose,
    in_service_namespace="",
    in_robot_tags=None,
    in_robot_json="",
    in_timeout=5.0,
):
    assert len(in_robot_model) > 0
    assert len(in_robot_name) > 0
    node = rclpy.create_node(f"spawn_{in_robot_name}")
    cli = wait_for_service(
        node, SpawnEntity, in_service_namespace + "/" + SERVICE_NAME_SPAWN_ENTITY
    )
    if not cli.service_is_ready():
        return False

    # Prepare SpawnEntity request
    req = SpawnEntity.Request()
    req.xml = in_robot_model
    req.robot_namespace = in_robot_namespace
    req.state = EntityState()
    req.state.name = in_robot_name
    req.state.reference_frame = in_robot_ref_frame
    req.state.pose = in_robot_pose
    req.tags = in_robot_tags
    req.json_parameters = in_robot_json

    # Async invoke SpawnEntity service
    future = cli.call_async(req)
    try:
        start = time.time()
        while (time.time() - start) < in_timeout:
            rclpy.spin_once(node)
            if future.done():
                break
            time.sleep(0.5)
    finally:
        node.destroy_node()
    return True


TOPIC_NAME_CMD_VEL = "cmd_vel"
PUBLISHING_NUM = 1
PUBLISHING_FREQ = 10


class CmdVelPublisher(Node):
    def __init__(
        self,
        in_robot_namespace: str = "",
        publishing_num=PUBLISHING_NUM,
        publishing_freq=PUBLISHING_FREQ,
        topic_name=TOPIC_NAME_CMD_VEL,
        in_robot_twist: Twist = Twist(),
    ):
        super().__init__(node_name="cmd_vel_publisher", namespace=in_robot_namespace)
        self._cmd_vel_topic = topic_name
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._twist_pub = self.create_publisher(
            Twist, self._cmd_vel_topic, qos_profile=latching_qos
        )
        self._twist = in_robot_twist

        if publishing_num > 0 and publishing_freq > 0:
            self._publishing_num = publishing_num
            self._twist_pub_count = 0
            self._is_pub_finished = False

            self._executor = SingleThreadedExecutor(context=self.context)
            self._executor.add_node(self)

            self._timer = self.create_timer(1.0 / publishing_freq, self.twist_robot)

            while rclpy.ok() and not self._is_pub_finished:
                self._executor.spin_once(timeout_sec=0)
            print(f"Finished publishing Twist to {self.get_full_topic_name()}")

    def get_full_topic_name(self):
        return f"{self.get_namespace()}/{self._cmd_vel_topic}"

    def __enter__(self):
        return self

    def __exit__(self, exep_type, exep_value, trace):
        if exep_type is not None:
            raise Exception("Exception occured, value: ", exep_value)
        self._executor.shutdown(timeout_sec=0.5)
        self.destroy_node()

    async def twist_robot(self):
        if self._twist_pub_count < self._publishing_num or self._publishing_num <= 0:
            self.get_logger().info(
                f"Publishing to {self.get_full_topic_name()}: {self._twist}"
            )
            self.pub(self._twist)
            self._twist_pub_count += 1
        else:
            self._is_pub_finished = True
            self._timer.cancel()
        await asyncio.sleep(0)

    def pub(self, twist=None):
        if twist is None:
            twist = self._twist
        self._twist_pub.publish(twist)

    def wait_for_robot_twisted(
        self,
        in_robot_name,
        in_robot_prev_pose,
        in_timeout=5.0,
        publish_twist_on_fail=True,
    ):
        assert (
            self._is_pub_finished
        ), f"Twist has not been published to {self.get_full_topic_name()}"

        start_time = time.time()
        while (time.time() - start_time) < in_timeout:
            time.sleep(1.0)
            is_robot_found, robot_pose = wait_for_spawned_entity(
                in_robot_name, in_timeout=5.0
            )
            assert (
                is_robot_found
            ), f"wait_for_robot_twisted(): {in_robot_name} unavailable!"
            if robot_pose != in_robot_prev_pose:
                return True
            elif publish_twist_on_fail:
                self.pub()

        return False
