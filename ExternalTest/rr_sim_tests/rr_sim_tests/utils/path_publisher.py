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

PUBLISHING_NUM = 1
PUBLISHING_FREQ = 10
TOPIC_NAME_PATH = 'path'
class PathPublisher(Node):
    def __init__(self, in_robot_namespace:str = '', in_publishing_num = PUBLISHING_NUM, in_publishing_freq = PUBLISHING_FREQ,
                 in_topic_name = TOPIC_NAME_PATH, in_robot_path:Path = Path()):
        super().__init__(node_name='path_publisher', namespace=in_robot_namespace)
        self._path_topic = in_topic_name
        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self._path_pub = self.create_publisher(Path, self._path_topic, qos_profile=latching_qos)
        self._path = in_robot_path
        self._pose_index = 0

        self._publishing_num = in_publishing_num
        self._path_pub_count = 0
        self._is_pub_finished = False

        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(self)
        self._timer = self.create_timer(1.0/in_publishing_freq, self.publish_path)

        while rclpy.ok() and not self._is_pub_finished:
            self._executor.spin_once(timeout_sec = 0)
        print(f'--> Finished publishing Path to {self.get_full_topic_name()} of {len(self._path.poses)} waypoints')

    def get_full_topic_name(self):
        return f'{self.get_namespace()}/{self._path_topic}'

    def __enter__(self):
        return self

    def __exit__(self, exep_type, exep_value, trace):
        if exep_type is not None:
            raise Exception('Exception occured, value: ', exep_value)
        self._executor.shutdown(timeout_sec = 0.5)
        self.destroy_node()
        
    async def publish_path(self):
        if (self._path_pub_count < self._publishing_num or self._publishing_num <= 0):
            self.get_logger().info(f'Publishing to {self.get_full_topic_name()}: {self._path}')
            self._path_pub.publish(self._path)
            self._path_pub_count += 1
        else:
            self._is_pub_finished = True
            self._timer.cancel()
        await asyncio.sleep(0)

    def wait_for_robot_path_followed(self, in_robot_name, in_robot_next_target_pose, in_timeout=20.0):
        assert self._is_pub_finished, f'Path has not been published to {self.get_full_topic_name()}'

        # Wait a while for robot to receive Path message
        start_time = time.time()
        while ((time.time() - start_time) < in_timeout):
            time.sleep(3.0)
            is_robot_found, robot_pose = wait_for_spawned_entity(in_robot_name, in_timeout)
            assert is_robot_found, f'wait_for_robot_path_followed(): {in_robot_name} unavailable!'
            if (robot_pose == in_robot_next_target_pose):
                print(f'{in_robot_name} has passed waypoint[{self._pose_index}]: {robot_pose}')
                self._pose_index += 1

                if (self._pose_index < len(self._path.poses)):
                    return self.wait_for_robot_path_followed(self, in_robot_name, self._path.poses[self._pose_index].pose)
                else:
                    return True

        return False