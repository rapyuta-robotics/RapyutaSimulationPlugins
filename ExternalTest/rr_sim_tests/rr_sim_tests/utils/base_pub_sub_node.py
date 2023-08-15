#! /usr/bin/env python3
# Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

# rclpy
import rclpy

# other ros
from rclpy.node import Node


class BasePubSubNode(Node):
    def __init__(
        self,
        in_namespace: str,
        in_topic_name:str
    ):
        super().__init__(node_name=f"{in_topic_name.replace('/','_')}_{type(self).__name__}", namespace=in_namespace)

        # https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/node_impl.hpp
        # https://github.com/ros2/rclcpp/issues/1767
        self._full_topic_name = None
        if in_topic_name.startswith('/'):
            self._full_topic_name = in_topic_name
        elif in_topic_name.startswith('~/'):
            self._full_topic_name = f"{in_topic_name.replace('~', self.get_effective_namespace(), 1)}"
        else:
            ns = '' if in_namespace == '' else self.get_namespace()
            self._full_topic_name = f'{ns}/{in_topic_name}'

    @property
    def full_topic(self):
        return self._full_topic_name
