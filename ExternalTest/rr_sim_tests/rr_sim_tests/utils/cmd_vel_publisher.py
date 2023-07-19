#! /usr/bin/env python3
# Copyright 2020-2023 Rapyuta Robotics Co., Ltd.

import asyncio

# rclpy
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

# other ros
from geometry_msgs.msg import Twist

# rr_sim_tests
from rr_sim_tests.utils.base_publisher import BasePublisher, PUBLISHER_DEFAULT_QOS

TOPIC_NAME_CMD_VEL = "cmd_vel"
CMD_VEL_PUBLISH_NUM = 5

# Note: [CMD_VEL_PUBLISHING_FREQ] here depends on [UE_FRAME_RATE & UE_DEFAULT_RTF] in such a way that the publishing 
# should be timely enough for Sim to receive the msg
CMD_VEL_PUBLISH_FREQ = 1.0 #Hz

class CmdVelPublisher(BasePublisher):
    def __init__(
        self,
        in_robot_namespace: str = "",
        in_topic_name=TOPIC_NAME_CMD_VEL,
        in_publish_num=CMD_VEL_PUBLISH_NUM,
        in_publish_freq=CMD_VEL_PUBLISH_FREQ,
        in_robot_twist: Twist = Twist(),
        in_auto_publish=True
    ):
        super().__init__(in_robot_namespace, in_topic_name, in_publish_num, in_publish_freq, in_auto_publish)
        self._publisher = self.create_publisher(
            Twist, self._full_topic_name, qos_profile=PUBLISHER_DEFAULT_QOS
        )
        self._pub_data = in_robot_twist
