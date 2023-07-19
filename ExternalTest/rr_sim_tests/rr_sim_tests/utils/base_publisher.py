#! /usr/bin/env python3
# Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

import asyncio
from threading import Thread

# rclpy
import rclpy

# other ros
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

# Default number of publishing times, if <=0 will keep publishing until forced to stop
PUBLISHER_DEFAULT_PUB_NUM = 1

# Note: [PUBLISHER_DEFAULT_FREQ] here depends on [UE_FRAME_RATE & UE_DEFAULT_RTF] in such a way that the publishing 
# should be timely enough for Sim to receive the msg
PUBLISHER_DEFAULT_FREQ = 1.0 #Hz
PUBLISHER_DEFAULT_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

class BasePublisher(Node):
    def __init__(
        self,
        in_robot_namespace: str,
        in_topic_name:str,
        in_publish_num=PUBLISHER_DEFAULT_PUB_NUM,
        in_publish_freq=PUBLISHER_DEFAULT_FREQ,
        in_auto_publish=True
    ):
        super().__init__(node_name=f"{in_topic_name.replace('/','_')}_publisher", namespace=in_robot_namespace)

        # https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/node_impl.hpp
        # https://github.com/ros2/rclcpp/issues/1767
        self._full_topic_name = None
        if in_topic_name.startswith('/') or in_topic_name.startswith('~'):
            self._full_topic_name = in_topic_name
        else:
            ns = '' if in_robot_namespace == '' else self.get_namespace()
            self._full_topic_name = f'{ns}/{in_topic_name}'

        self._executor = None
        self._pub_thread = None

        self._publisher = None
        self._pub_data = None
        self._pub_count = 0
        self._pub_freq = in_publish_freq
        self._auto_pub = in_auto_publish
        self._is_pub_finished = False

        # Setup auto loop-publishing running async in thread
        self._publishing_num = in_publish_num
        if in_publish_freq > 0:
            self._executor = SingleThreadedExecutor(context=self.context)
            self._executor.add_node(self)

            self._pub_time_step = 1.0 / in_publish_freq
            self._timer = self.create_timer(self._pub_time_step, self.async_publish)
        elif in_auto_publish:
            self._publishing_num = 1

    @property
    def full_topic(self):
        return self._full_topic_name

    @property
    def pub_freq(self):
        return self._pub_freq
    
    @property
    def data(self):
        return self._pub_data

    @property
    def is_forever_publishing(self):
        return self._publishing_num <= 0

    def __enter__(self):
        # Keep publishing until Node is shutdown Or Wait for publishing finish
        if self.is_forever_publishing:
            self._pub_thread = Thread(target=self.spin, args=(False,), daemon=True)
            self._pub_thread.start()
        elif self._auto_pub:
            self.wait_for_pub_finish()
        return self

    def __exit__(self, exep_type, exep_value, trace):
        if exep_type is not None:
            raise Exception("Exception occured, value: ", exep_value)
        self._timer.cancel()
        self._executor.shutdown(timeout_sec=1.0)
        self.destroy_node()

        if self._pub_thread:
            try:
                self._pub_thread.join(timeout=1.0)
            except KeyboardInterrupt:
                pass

    def spin(self, in_spin_once=False):
        if self._executor:
            try:
                if in_spin_once:
                    self._executor.spin_once(timeout_sec=1.0)
                else:
                    self._executor.spin()
            except rclpy.executors.ExternalShutdownException:
                pass
        else:
            assert False, f"No executor available, make sure {self._pub_freq} > 0!"

    def wait_for_pub_finish(self):
        while rclpy.ok() and not self._is_pub_finished:
            self.spin(True)
        print(f"Finished publishing to {self._full_topic_name}")

    async def async_publish(self):
        if self.is_forever_publishing or (self._pub_count < self._publishing_num):
            self.publish(self._pub_data)
            self._pub_count += 1
        else:
            self._is_pub_finished = True
            self._timer.cancel()
        await asyncio.sleep(0)

    def publish(self, in_data=None):
        if in_data is None:
            in_data = self._pub_data
        assert(in_data is not None)

        assert(self._publisher)
        self._publisher.publish(in_data)
        self.get_logger().info(
            f"Publishing to {self._full_topic_name}: {in_data}"
        )


