#!/usr/bin/env python
# Copyright (c) 2020 - The Procedural Generation for Gazebo authors
# For information on the respective copyright owner see the NOTICE file
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import unittest
from pcg_gazebo.parsers import parse_sdf
from pcg_gazebo.simulation import add_custom_gazebo_resource_path, \
    Actor

CUR_DIR = os.path.dirname(os.path.abspath(__file__))


class TestActor(unittest.TestCase):
    def test_actor_walking(self):
        add_custom_gazebo_resource_path(os.path.join(
            CUR_DIR,
            'gazebo_models'))
        sdf = parse_sdf(os.path.join(
            CUR_DIR,
            'gazebo_models',
            'test_actor_walking',
            'model.sdf'))
        self.assertIsNotNone(sdf)
        self.assertIsNotNone(sdf.actors)
        self.assertEqual(len(sdf.actors), 1)

        actor = Actor.from_sdf(sdf.actors[0])
        self.assertIsNotNone(actor)

        self.assertEqual(actor.name, 'actor')
        self.assertEqual(actor.pose.position[0], 0)
        self.assertEqual(actor.pose.position[1], 0)
        self.assertEqual(actor.pose.position[2], 0)

        self.assertEqual(len(actor.animations), 1)
        self.assertEqual(actor.animations[0].name, 'walking')
        self.assertEqual(
            actor.animations[0].filename,
            'model://test_actor_walking/meshes/walk.dae')
        self.assertEqual(actor.animations[0].scale, 1)
        self.assertEqual(actor.animations[0].interpolate_x, True)

        self.assertEqual(len(actor.script.trajectories), 1)
        self.assertEqual(actor.script.trajectories[0].id, 0)
        self.assertEqual(actor.script.trajectories[0].type, 'walking')

        self.assertEqual(len(actor.script.trajectories[0].waypoints), 32)

    def test_actor_relative_paths(self):
        add_custom_gazebo_resource_path(os.path.join(
            CUR_DIR,
            'gazebo_models'))
        sdf = parse_sdf(os.path.join(
            CUR_DIR,
            'gazebo_models',
            'test_actor_relative_paths',
            'model.sdf'))
        self.assertIsNotNone(sdf)
        self.assertIsNotNone(sdf.actors)
        self.assertEqual(len(sdf.actors), 1)

        actor = Actor.from_sdf(sdf.actors[0])
        self.assertIsNotNone(actor)

        self.assertEqual(actor.name, 'actor_test')
        self.assertEqual(actor.pose.position[0], 0)
        self.assertEqual(actor.pose.position[1], 0)
        self.assertEqual(actor.pose.position[2], 1)

        self.assertEqual(len(actor.animations), 8)
        self.assertEqual(len(actor.script.trajectories), 13)


if __name__ == '__main__':
    unittest.main()
