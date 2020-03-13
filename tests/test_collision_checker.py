#!/usr/bin/env python
# Copyright (c) 2019 - The Procedural Generation for Gazebo authors
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
import numpy as np
from pcg_gazebo.generators.creators import sphere
from pcg_gazebo.generators import CollisionChecker, SingletonCollisionChecker


class TestCollisionChecker(unittest.TestCase):
    def test_reset_scenario(self):
        main_sphere = sphere(mass=1, radius=0.5, name='sphere')

        cc = CollisionChecker()
        self.assertEqual(cc.n_fixed_models, 0)
        self.assertEqual(cc.n_meshes, 0)

        cc.add_model(main_sphere)
        self.assertEqual(cc.n_fixed_models, 0)
        self.assertEqual(cc.n_meshes, 1)

        cc.reset_scenario()
        self.assertEqual(cc.n_fixed_models, 0)
        self.assertEqual(cc.n_meshes, 0)

        cc.add_fixed_model(main_sphere)
        self.assertEqual(cc.n_fixed_models, 1)
        self.assertEqual(cc.n_meshes, 1)

        cc.reset_scenario()
        self.assertEqual(cc.n_fixed_models, 1)
        self.assertEqual(cc.n_meshes, 0)

        cc.reset_to_fixed_model_scenario()
        self.assertEqual(cc.n_fixed_models, 1)
        self.assertEqual(cc.n_meshes, 1)

        # Try to add a ground plane model
        main_sphere.is_ground_plane = True
        cc.add_fixed_model(main_sphere)
        self.assertEqual(cc.n_fixed_models, 1)
        self.assertEqual(cc.n_meshes, 1)

    def test_singleton_collision_checker(self):
        cc1 = SingletonCollisionChecker.get_instance()
        self.assertIsNotNone(cc1)
        cc2 = SingletonCollisionChecker.get_instance()
        self.assertIsNotNone(cc2)

        self.assertEqual(cc1.n_fixed_models, 0)
        self.assertEqual(cc2.n_fixed_models, 0)

        main_sphere = sphere(mass=1, radius=0.5, name='sphere')

        cc1.add_fixed_model(main_sphere)

        self.assertEqual(cc1.n_fixed_models, 1)
        self.assertEqual(cc2.n_fixed_models, 1)

        self.assertEqual(
            cc1.fixed_models[0].to_sdf(),
            cc2.fixed_models[0].to_sdf())

    def test_collision_check(self):
        main_sphere = sphere(mass=1, radius=0.5, name='sphere')
        self.assertIsNotNone(main_sphere)

        cc = CollisionChecker()
        cc.add_fixed_model(main_sphere)
        self.assertEqual(cc.n_fixed_models, 1)
        self.assertEqual(cc.n_meshes, 1)

        centers = np.random.uniform(0, 3, size=(20, 3))

        for k in range(centers.shape[0]):
            c = centers[k, :]
            radius = np.random.uniform(0.5, 1)
            model = sphere(mass=1, radius=radius, name='test',
                           pose=[i for i in c] + [0, 0, 0])
            self.assertIsNotNone(model)

            if np.linalg.norm(c) <= 0.5:
                self.assertTrue(cc.check_collision_with_current_scene(model))
            # elif np.linalg.norm(c) - radius <= 0.5:
            #     self.assertTrue(cc.check_collision_with_current_scene(model))
            # else:
            #     self.assertFalse(cc.check_collision_with_current_scene(model))

    def test_mesh_within(self):
        main_sphere = sphere(mass=1, radius=1, name='sphere')

        centers = np.random.uniform(0, 0.2, size=(20, 3))

        cc = CollisionChecker()
        cc.add_fixed_model(main_sphere)
        self.assertEqual(cc.n_fixed_models, 1)
        self.assertEqual(cc.n_meshes, 1)

        for k in range(centers.shape[0]):
            c = centers[k, :]
            radius = np.random.uniform(0.01, 0.1)
            model = sphere(mass=1, radius=radius, name='test',
                           pose=[i for i in c] + [0, 0, 0])
            self.assertIsNotNone(model)

            self.assertTrue(cc.check_collision_with_current_scene(model))

    def test_mesh_around_another(self):
        main_sphere = sphere(mass=1, radius=0.1, name='sphere')

        centers = np.random.uniform(0, 0.2, size=(20, 3))

        cc = CollisionChecker()
        cc.add_fixed_model(main_sphere)
        self.assertEqual(cc.n_fixed_models, 1)
        self.assertEqual(cc.n_meshes, 1)

        for k in range(centers.shape[0]):
            c = centers[k, :]
            radius = np.random.uniform(0.5, 1.0)
            model = sphere(mass=1, radius=radius, name='test',
                           pose=[i for i in c] + [0, 0, 0])
            self.assertIsNotNone(model)

            self.assertTrue(cc.check_collision_with_current_scene(model))


if __name__ == '__main__':
    unittest.main()
