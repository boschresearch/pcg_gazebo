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
import unittest
from pcg_gazebo.generators.creators import extrude, box
from pcg_gazebo.generators.shapes import random_rectangles
from pcg_gazebo.simulation import World


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
            polygon=random_rectangles(),
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

        bounds = walls.get_bounds()
        area = (bounds[1, 0] - bounds[0, 0]) * \
            (bounds[1, 1] - bounds[0, 1])

        self.assertIsNotNone(free_space_polygon)
        self.assertLessEqual(free_space_polygon.area, area)

    def test_find_random_spots(self):
        walls = extrude(
            polygon=random_rectangles(),
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

        poses, free_space_polygon = \
            world.get_random_free_spots(
                box_model,
                n_spots=5,
                ground_plane_models='walls',
                dofs=['x', 'y', 'yaw'],
                return_free_polygon=True
            )

        self.assertEqual(len(poses), 5)
        self.assertIsNotNone(free_space_polygon)

        bounds = walls.get_bounds()
        area = (bounds[1, 0] - bounds[0, 0]) * \
            (bounds[1, 1] - bounds[0, 1])

        self.assertIsNotNone(free_space_polygon)
        self.assertLessEqual(free_space_polygon.area, area)


if __name__ == '__main__':
    unittest.main()
