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
from pcg_gazebo.generators.creators import extrude, box
from pcg_gazebo.generators.shapes import random_rectangle
from pcg_gazebo.simulation import World
from pcg_gazebo.parsers import parse_sdf
from pcg_gazebo import random


class TestWorld(unittest.TestCase):
    def test_compute_empty_world_free_space(self):
        world = World()
        with self.assertRaises(AssertionError):
            free_space_polygon = world.get_free_space_polygon()

        free_space_polygon = world.get_free_space_polygon(
            x_limits=[-100, 100],
            y_limits=[-100, 100]
        )

        self.assertIsNotNone(free_space_polygon)
        self.assertEqual(free_space_polygon.area, 200 * 200)

    def test_compute_free_space(self):
        walls = extrude(
            polygon=random_rectangle(
                delta_x_min=10,
                delta_y_min=10),
            extrude_boundaries=True,
            height=2,
            thickness=0.1
        )
        walls.pose = [0, 0, 1, 0, 0, 0]
        world = World()
        world.add_model(tag='walls', model=walls)

        self.assertEqual(world.n_models, 1)

        free_space_polygon = \
            world.get_free_space_polygon(
                ground_plane_models=['walls'])

        self.assertIsNotNone(free_space_polygon)
        self.assertGreater(free_space_polygon.area, 0)

    def test_find_random_spots(self):
        walls = extrude(
            polygon=random_rectangle(
                delta_x_min=10,
                delta_y_min=10),
            extrude_boundaries=True,
            height=2,
            thickness=0.1
        )
        walls.pose = [0, 0, 1, 0, 0, 0]
        world = World()
        world.add_model(tag='walls', model=walls)

        self.assertEqual(world.n_models, 1)

        box_model = box(
            size=[0.05, 0.05, 0.05], pose=[0, 0, 0.025, 0, 0, 0])

        n_spots = random.randint(2, 5)
        poses, free_space_polygon = \
            world.get_random_free_spots(
                box_model,
                n_spots=n_spots,
                ground_plane_models=['walls'],
                dofs=['x', 'y', 'yaw'],
                return_free_polygon=True
            )

        self.assertEqual(len(poses), n_spots)
        self.assertIsNotNone(free_space_polygon)

        self.assertIsNotNone(free_space_polygon)
        self.assertGreater(free_space_polygon.area, 0)

    def test_parse_world_files(self):
        world_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'worlds'
        )
        for item in os.listdir(world_dir):
            world_file = os.path.join(world_dir, item)
            if not os.path.isfile(world_file) or \
                    not world_file.endswith('.world'):
                continue
            sdf = parse_sdf(world_file)
            self.assertIsNotNone(sdf)
            self.assertEqual(sdf.xml_element_name, 'sdf')
            self.assertIsNotNone(
                sdf.world,
                'No world element was parsed from file {}'.format(world_file))

            if sdf.world.includes is None:
                world = World.from_sdf(sdf)
                self.assertIsNotNone(world)

                w_sdf = world.to_sdf(type='sdf')
                self.assertIsNotNone(w_sdf.world)

                for tag in sdf.world.children:
                    if sdf.world.children[tag] is None:
                        continue

                    if isinstance(sdf.world.children[tag], list):
                        if tag == 'model':
                            for model in sdf.world.children[tag]:
                                self.assertTrue(world.model_exists(model.name))
                        if tag == 'light':
                            for light in sdf.world.children[tag]:
                                self.assertTrue(world.light_exists(light.name))
                    elif tag == 'physics':
                        self.assertEqual(
                            sdf.world.physics.type,
                            w_sdf.world.physics.type)
                        for physics_tag in sdf.world.physics.children:
                            obj = getattr(sdf.world.physics, physics_tag)
                            self.assertIn(
                                physics_tag,
                                w_sdf.world.physics.children)
                            if obj.has_value():
                                self.assertEqual(
                                    obj,
                                    getattr(w_sdf.world.physics, physics_tag))
                    else:
                        obj = getattr(sdf.world, tag)
                        if obj.has_value():
                            self.assertEqual(obj, getattr(w_sdf.world, tag))


if __name__ == '__main__':
    unittest.main()
