#! /usr/bin/env python3
# Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

import asyncio
import time

# rclpy
import rclpy

# other ros
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy, QoSProfile

# rr_sim_tests
from rr_sim_tests.utils.wait_for_topics import WaitForTopics
from rr_sim_tests.utils.base_pub_sub_node import BasePubSubNode

# Data polling rate
SUBSCRIBER_DEFAULT_RATE = 10.0 #Hz
SUBSCRIBER_DEFAULT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
    depth=10
)

class BaseSubscriber(BasePubSubNode):
    def __init__(self, in_robot_namespace: str, 
                       in_topic_name: str,
                       in_subscriber_rate=SUBSCRIBER_DEFAULT_RATE):
        super().__init__(in_namespace=in_robot_namespace, in_topic_name=in_topic_name)
        self._subscriber = None
        self._topic_data_arrived = False
        self._received_data = None

        self._time_step = 1.0 / in_subscriber_rate

    def __enter__(self):
        return self

    def __exit__(self, exep_type, exep_value, trace):
        if exep_type is not None:
            raise Exception("Exception occured, value: ", exep_value)
        self.destroy_node()

    @property
    def full_topic(self):
        return self._full_topic_name

    @property
    def received_data(self):
        return self._received_data

    @property
    def has_topic_data_arrived(self):
        return self._topic_data_arrived

    def wait_for_topic_data(self, in_timeout=5.0):
        start_time = time.time()
        while (not self._topic_data_arrived) and (
            (time.time() - start_time) < in_timeout
        ):
            # [on_data_arrived] callback is triggered here-in upon msg coming
            rclpy.spin_once(self, timeout_sec=self._time_step)
        return self._topic_data_arrived
