#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import unittest

import launch
import launch_testing.actions
import launch_testing.markers

import rclpy

from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler

from rr_sim_tests.utils.wait_for_spawned_entity import wait_for_spawned_entity
from rr_sim_tests.utils.utils import spawn_robot

import pytest

"""
Test robot spawning
"""
LAUNCH_ARG_ROBOT_MODEL = 'robot_model'
LAUNCH_ARG_ROBOT_NAMESPACE = 'robot_namespace'
LAUNCH_ARG_ROBOT_NAME = 'robot_name'
LAUNCH_ARG_ROBOT_REF_FRAME = 'robot_ref_frame'
LAUNCH_ARG_ROBOT_POS = 'robot_pos'
LAUNCH_ARG_ROBOT_ROT = 'robot_rot'
LAUNCH_ARG_ROBOT_TAGS = 'robot_tags'
LAUNCH_ARG_ROBOT_JSON = 'robot_json'

SERVICE_NAME_SPAWN_ENTITY = 'SpawnEntity'

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot_model = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_MODEL, default='')
    robot_namespace = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_NAMESPACE, default='')
    robot_name = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_NAME, default='')
    robot_ref_frame = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_REF_FRAME, default='')
    robot_pos = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_POS, default='0.0, 0.0, 0.0')
    robot_rot = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_ROT, default='0.0, 0.0, 0.0')
    robot_tags = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_TAGS, default='')
    robot_json = launch.substitutions.LaunchConfiguration(LAUNCH_ARG_ROBOT_JSON, default='')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_MODEL,
            default_value=robot_model,
            description='Robot model (rr_pa_amr, amr, gen3, etc.)'),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_NAME,
            default_value=robot_name,
            description='Robot unique name'),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_NAMESPACE,
            default_value=robot_namespace,
            description='Robot namespace'),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_REF_FRAME,
            default_value=robot_ref_frame,
            description='Robot reference frame'),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_POS,
            default_value=robot_pos,
            description="Robot initial position (xyz). Eg:'0.0, 0.0, 0.0'"),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_ROT,
            default_value=robot_rot,
            description="Robot initial rotation (roll, pitch, yaw). Eg:'0.0, 0.0, 0.0'"),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_TAGS,
            default_value=robot_tags,
            description="Robot tags separated by ','. Eg:'map_origin'"),
        launch.actions.DeclareLaunchArgument(
            LAUNCH_ARG_ROBOT_JSON,
            default_value=robot_tags,
            description="Robot json configs"),
        launch_testing.actions.ReadyToTest()
    ])
class TestRobotSpawn(unittest.TestCase):
    def test_spawn_robot(self, proc_output, test_args):
        rclpy.init()

        arghas = lambda arg: arg in test_args
        argstr = lambda arg: str(test_args[arg]) if arg in test_args else ''
        
        assert(arghas(LAUNCH_ARG_ROBOT_MODEL))
        assert(arghas(LAUNCH_ARG_ROBOT_NAME))
        robot_name = argstr(LAUNCH_ARG_ROBOT_NAME)
        robot_pose = Pose()
        
        pos = argstr(LAUNCH_ARG_ROBOT_POS).split(',')
        robot_pose.position.x = float(pos[0])
        robot_pose.position.y = float(pos[1])
        robot_pose.position.z = float(pos[2])
        
        rot = argstr(LAUNCH_ARG_ROBOT_ROT).split(',')
        q = quaternion_from_euler(float(rot[0]), float(rot[1]), float(rot[2]))
        robot_pose.orientation.x = q[0]
        robot_pose.orientation.y = q[1]
        robot_pose.orientation.z = q[2]
        robot_pose.orientation.w = q[3]

        robot_tags = argstr(LAUNCH_ARG_ROBOT_TAGS).split(',')
        robot_json = argstr(LAUNCH_ARG_ROBOT_JSON)
        assert spawn_robot(argstr(LAUNCH_ARG_ROBOT_MODEL),
                           robot_name,
                           argstr(LAUNCH_ARG_ROBOT_NAMESPACE),
                           argstr(LAUNCH_ARG_ROBOT_REF_FRAME),
                           robot_pose,
                           in_robot_tags=robot_tags,
                           in_robot_json=robot_json)
        is_robot_spawned, _ = wait_for_spawned_entity(robot_name, 8.0)
        assert is_robot_spawned, f'{robot_name} failed being spawned!'
        rclpy.shutdown()
