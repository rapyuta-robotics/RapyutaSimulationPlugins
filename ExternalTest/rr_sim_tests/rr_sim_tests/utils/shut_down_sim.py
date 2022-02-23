#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

from action_msgs.msg import GoalStatus
from ue_msgs.action import SimShutDown

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

ACTION_NAME_SIM_SHUT_DOWN = '/SimShutDown'

class SimShutDownActionClient(Node):
    def __init__(self):
        super().__init__('sim_shut_down_action_client')
        self._action_client = ActionClient(self, SimShutDown, ACTION_NAME_SIM_SHUT_DOWN)

    def __enter__(self):
        return self

    def __exit__(self, exep_type, exep_value, trace):
        if exep_type is not None:
            raise Exception('Exception occured, value: ', exep_value)
        self._action_client.destroy()
        self.destroy_node()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.sequence))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.sequence))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

    def wait_for_action_server(self, in_timeout=10.0):
        self.get_logger().info(f'Waiting for [{ACTION_NAME_SIM_SHUT_DOWN}] action server...')
        assert(self._action_client.wait_for_server(in_timeout))

    def shut_down_sim_async(self, in_timeout=10.0):
        assert(self.wait_for_action_server(in_timeout))

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(
            SimShutDown.Goal(),
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def shut_down_sim(self, in_timeout=10.0):
        assert(self.wait_for_action_server(in_timeout))

        # Send goal
        self.get_logger().info('Sending SimShutDown goal request...')
        goal_future = self._action_client.send_goal_async(SimShutDown.Goal())
        rclpy.spin_until_future_complete(self, goal_future)
        assert(goal_future.done())
        goal_handle = goal_future.result()

        # Get goal result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        assert(result_future.done())

