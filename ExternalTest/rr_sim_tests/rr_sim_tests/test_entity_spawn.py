#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

import time
import unittest

import launch
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from geometry_msgs.msg import Pose
from rr_sim_tests.utils.utils import spawn_entity
from rr_sim_tests.utils.wait_for_spawned_entity import wait_for_spawned_entity
from tf_transformations import quaternion_from_euler

"""
Test entity spawning
"""
LAUNCH_ARG_ENTITY_MODEL = "entity_model"
LAUNCH_ARG_ENTITY_NAMESPACE = "entity_namespace"
LAUNCH_ARG_ENTITY_NAME = "entity_name"
LAUNCH_ARG_ENTITY_REF_FRAME = "entity_ref_frame"
LAUNCH_ARG_ENTITY_POS = "entity_pos"
LAUNCH_ARG_ENTITY_ROT = "entity_rot"
LAUNCH_ARG_ENTITY_TAGS = "entity_tags"
LAUNCH_ARG_ENTITY_JSON = "entity_json"

SERVICE_NAME_SPAWN_ENTITY = "SpawnEntity"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    entity_model = launch.substitutions.LaunchConfiguration(
        LAUNCH_ARG_ENTITY_MODEL, default=""
    )
    entity_namespace = launch.substitutions.LaunchConfiguration(
        LAUNCH_ARG_ENTITY_NAMESPACE, default=""
    )
    entity_name = launch.substitutions.LaunchConfiguration(
        LAUNCH_ARG_ENTITY_NAME, default=""
    )
    entity_ref_frame = launch.substitutions.LaunchConfiguration(
        LAUNCH_ARG_ENTITY_REF_FRAME, default=""
    )
    entity_pos = launch.substitutions.LaunchConfiguration(
        LAUNCH_ARG_ENTITY_POS, default="0.0, 0.0, 0.0"
    )
    entity_rot = launch.substitutions.LaunchConfiguration(
        LAUNCH_ARG_ENTITY_ROT, default="0.0, 0.0, 0.0"
    )
    entity_tags = launch.substitutions.LaunchConfiguration(
        LAUNCH_ARG_ENTITY_TAGS, default=""
    )
    entity_json = launch.substitutions.LaunchConfiguration(
        LAUNCH_ARG_ENTITY_JSON, default=""
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                LAUNCH_ARG_ENTITY_MODEL,
                default_value=entity_model,
                description="Entity model (rr_pa_amr, amr, box, etc.)",
            ),
            launch.actions.DeclareLaunchArgument(
                LAUNCH_ARG_ENTITY_NAME,
                default_value=entity_name,
                description="Entity unique name",
            ),
            launch.actions.DeclareLaunchArgument(
                LAUNCH_ARG_ENTITY_NAMESPACE,
                default_value=entity_namespace,
                description="Entity namespace",
            ),
            launch.actions.DeclareLaunchArgument(
                LAUNCH_ARG_ENTITY_REF_FRAME,
                default_value=entity_ref_frame,
                description="Entity reference frame",
            ),
            launch.actions.DeclareLaunchArgument(
                LAUNCH_ARG_ENTITY_POS,
                default_value=entity_pos,
                description="Entity initial position (xyz). Eg:'0.0, 0.0, 0.0'",
            ),
            launch.actions.DeclareLaunchArgument(
                LAUNCH_ARG_ENTITY_ROT,
                default_value=entity_rot,
                description="Entity initial rotation (roll, pitch, yaw). Eg:'0.0, 0.0, 0.0'",
            ),
            launch.actions.DeclareLaunchArgument(
                LAUNCH_ARG_ENTITY_TAGS,
                default_value=entity_tags,
                description="Entity tags separated by ','. Eg:'map_origin'",
            ),
            launch.actions.DeclareLaunchArgument(
                LAUNCH_ARG_ENTITY_JSON,
                default_value=entity_json,
                description="Entity json configs",
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )

class TestEntitySpawn(unittest.TestCase):
    def test_spawn_entity(self, proc_output, test_args):
        rclpy.init()

        arghas = lambda arg: arg in test_args
        argstr = lambda arg: str(test_args[arg]) if arg in test_args else ""

        assert arghas(LAUNCH_ARG_ENTITY_MODEL)
        assert arghas(LAUNCH_ARG_ENTITY_NAME)
        entity_name = argstr(LAUNCH_ARG_ENTITY_NAME)
        entity_pose = Pose()

        pos = argstr(LAUNCH_ARG_ENTITY_POS).split(",")
        entity_pose.position.x = float(pos[0])
        entity_pose.position.y = float(pos[1])
        entity_pose.position.z = float(pos[2])

        rot = argstr(LAUNCH_ARG_ENTITY_ROT).split(",")
        q = quaternion_from_euler(float(rot[0]), float(rot[1]), float(rot[2]))
        entity_pose.orientation.x = q[0]
        entity_pose.orientation.y = q[1]
        entity_pose.orientation.z = q[2]
        entity_pose.orientation.w = q[3]

        entity_tags = argstr(LAUNCH_ARG_ENTITY_TAGS).split(",")
        entity_json = argstr(LAUNCH_ARG_ENTITY_JSON)
        entity_ref_frame = argstr(LAUNCH_ARG_ENTITY_REF_FRAME)
        assert spawn_entity(
            argstr(LAUNCH_ARG_ENTITY_MODEL),
            entity_name,
            argstr(LAUNCH_ARG_ENTITY_NAMESPACE),
            entity_ref_frame,
            entity_pose,
            in_entity_tags=entity_tags,
            in_entity_json=entity_json
        )

        is_entity_spawned, _ = wait_for_spawned_entity(entity_name, in_timeout=10.0, in_entity_ref_frame=entity_ref_frame)
        assert is_entity_spawned, f"{entity_name} failed being spawned!"
        rclpy.shutdown()
