#! /usr/bin/env python3
# Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

import asyncio
import time

# rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.executors import SingleThreadedExecutor

# other ros
from nav_msgs.msg import Path

# UE_msgs
from ue_msgs.msg import EntityState

# rr_sim_tests
from rr_sim_tests.utils.wait_for_spawned_entity import wait_for_spawned_entity
from rr_sim_tests.utils.base_publisher import BasePublisher, PUBLISHER_DEFAULT_QOS

TOPIC_NAME_PATH = 'path'
PATH_PUBLISH_NUM = 1
PATH_PUBLISH_FREQ = 10.0 #Hz
class PathPublisher(BasePublisher):
    def __init__(self, in_robot_namespace:str = '',
                 in_topic_name = TOPIC_NAME_PATH,
                 in_publish_num = PATH_PUBLISH_NUM,
                 in_publish_freq = PATH_PUBLISH_FREQ,
                 in_robot_path:Path = Path()):
        super().__init__(in_robot_namespace, in_topic_name, in_publish_num, in_publish_freq)
        self._publisher = self.create_publisher(
            Path, in_topic_name, qos_profile=PUBLISHER_DEFAULT_QOS
        )
        self._pub_data = in_robot_path
        self._pose_index = 0

    def wait_for_robot_waypoint_reached(self, in_robot_name, in_robot_next_target_pose, in_timeout=20.0):
        assert self._is_pub_finished, f'Path has not been published to {self._full_topic_name}'

        # Wait a while for robot to receive Path message
        start_time = time.time()
        while ((time.time() - start_time) < in_timeout):
            time.sleep(3.0)
            is_robot_found, robot_pose = wait_for_spawned_entity(in_robot_name, in_timeout)
            assert is_robot_found, f'wait_for_robot_waypoint_reached(): {in_robot_name} unavailable!'
            if (robot_pose == in_robot_next_target_pose):
                print(f'{in_robot_name} has passed waypoint[{self._pose_index}]: {robot_pose}')
                self._pose_index += 1

                if (self._pose_index < len(self._pub_data.poses)):
                    return self.wait_for_robot_waypoint_reached(self, in_robot_name, self._pub_data.poses[self._pose_index].pose)
                else:
                    return True

        return False