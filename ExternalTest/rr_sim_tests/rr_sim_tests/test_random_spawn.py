#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import os
import time
import random
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose, Twist
from tf_transformations import quaternion_from_euler

from rr_sim_tests.utils.utils import CmdVelPublisher
from rr_sim_tests.utils.utils import spawn_robot
from rr_sim_tests.utils.wait_for_spawned_entity import wait_for_spawned_entity

# todo do we get param without node?
class ParamsNode(Node):
    def __init__(self):
        super().__init__('params_node')
        self.declare_parameters(
        namespace='',
        parameters=[
            ('x_lim', [4.0, -4.0]),
            ('y_lim', [4.0, -4.0]),
            ('linear_vel_lim', [1.0, -1.0]),
            ('angular_vel_lim', [np.pi*0.5, -np.pi*0.5]),
            ('robot_name', None),
            ('service_namespace', '')
        ]
    )
def main(args=None):
    rclpy.init(args=args)

    param_node = ParamsNode()

    # robot_models = ['kinematic_burger', 'kinematic_waffle'] #kinematics model
    robot_models = ['burger', 'waffle'] #physics model

    # parameters
    x_lim = param_node.get_parameter('x_lim').value
    y_lim = param_node.get_parameter('y_lim').value
    linear_vel_lim = param_node.get_parameter('linear_vel_lim').value
    angular_vel_lim = param_node.get_parameter('angular_vel_lim').value
    robot_name = param_node.get_parameter('robot_name').value
    service_namespace = param_node.get_parameter('service_namespace').value

    # sanity check
    if robot_name is None:
        print('You need to provide robot name with `--ros-args -p robot_name:=<name>`')
        return

    is_robot_spawned, _ = wait_for_spawned_entity(robot_name, service_namespace, 10.0)
    if is_robot_spawned:
        print(robot_name + 'is already exists.')
        return

    # spawn robot
    robot_namespace = robot_name
    robot_model = random.choice(tuple(robot_models))
    reference_frame = ''
    robot_pose = Pose()

    res = True
    while res:
        q = quaternion_from_euler(0.0, 0.0, random.uniform(-np.pi, np.pi))
        robot_pose.position.x = float(random.uniform(x_lim[0], x_lim[1]))
        robot_pose.position.y = float(random.uniform(y_lim[0], y_lim[1]))
        robot_pose.position.z = 0.03
        robot_pose.orientation.x = q[0]
        robot_pose.orientation.y = q[1]
        robot_pose.orientation.z = q[2]
        robot_pose.orientation.w = q[3]
        is_srv_available = spawn_robot(robot_model, robot_name, robot_namespace, reference_frame, robot_pose, service_namespace)
        if is_srv_available:
            count = 0
            while count < 20:
                time.sleep(1)
                is_robot_spawned, _ = wait_for_spawned_entity(robot_name, service_namespace, 10.0)
                if is_robot_spawned:
                    res = False
                    break
                count += 1
    
    # publish cmd_vel
    while True:
        robot_twist = Twist()
        robot_twist.linear.x = random.uniform(linear_vel_lim[0], linear_vel_lim[1])
        robot_twist.angular.z = random.uniform(angular_vel_lim[0], angular_vel_lim[1])
        cmd_vel_publisher = CmdVelPublisher(in_robot_namespace=robot_name, publishing_freq=1, publishing_num=10, in_robot_twist=robot_twist)
        cmd_vel_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    print(sys.argv[0])
    main(sys.argv[:])
