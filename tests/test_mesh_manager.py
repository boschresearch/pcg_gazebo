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
from pcg_gazebo.collection_managers import MeshManager
from pcg_gazebo.utils import generate_random_string
from pcg_gazebo import random

CWD = os.path.dirname(os.path.abspath(__file__))


class TestMeshManager(unittest.TestCase):
    def test_add_box_primitive(self):
        mesh_manager = MeshManager.get_instance()
        mesh_manager.reset()
        self.assertEqual(len(mesh_manager.tags), 0)

        tag = mesh_manager.add(tag='box_1', type='box', size=[1, 1, 1])
        self.assertEqual(tag, 'box_1')
        tag = mesh_manager.add(tag='box_2', type='box', size=[2, 2, 2])
        self.assertEqual(tag, 'box_2')

        # Add box with same size of box_1
        tag = mesh_manager.add(type='box', size=[1, 1, 1])
        self.assertEqual(tag, 'box_1')

        tag = mesh_manager.find_by_parameters(
            type='box', size=[2, 2, 2])
        self.assertEqual(tag, 'box_2')

        # Let the manager generate the tag
        self.assertIsNotNone(mesh_manager.add(
            type='box', size=random.rand(3)))

    def test_add_wrong_box_input(self):
        mesh_manager = MeshManager.get_instance()
        mesh_manager.reset()
        self.assertEqual(len(mesh_manager.tags), 0)

        # Not specifying the type of the geometry
        self.assertIsNone(mesh_manager.add(size=[1, 1, 1]))

        wrong_sizes = [
            [-1, 1, 1],
            [2, 3],
            None,
            dict(),
            'a'
        ]
        # Entering wrong size vectors
        for size in wrong_sizes:
            with self.assertRaises(AssertionError):
                mesh_manager.add(type='box', size=size)

    def test_add_cylinder_primitive(self):
        mesh_manager = MeshManager.get_instance()
        mesh_manager.reset()
        self.assertEqual(len(mesh_manager.tags), 0)

        tag = mesh_manager.add(
            tag='cyl_1', type='cylinder', radius=1, height=1)
        self.assertEqual(tag, 'cyl_1')
        tag = mesh_manager.add(
            tag='cyl_2', type='cylinder', radius=2, height=2)
        self.assertEqual(tag, 'cyl_2')

        # Add cylinder with same size of cyl_1
        tag = mesh_manager.add(type='cylinder', radius=1, height=1)
        self.assertEqual(tag, 'cyl_1')

        tag = mesh_manager.find_by_parameters(
            type='cylinder', radius=2, height=2)
        self.assertEqual(tag, 'cyl_2')

        # Let the manager generate the tag
        self.assertIsNotNone(mesh_manager.add(
            type='cylinder',
            radius=random.rand(),
            height=random.rand()))

    def test_add_wrong_cylinder_input(self):
        mesh_manager = MeshManager.get_instance()
        mesh_manager.reset()
        self.assertEqual(len(mesh_manager.tags), 0)

        # Not specifying the type of the geometry
        self.assertIsNone(mesh_manager.add(radius=1, height=1))
        # Missing parameters
        self.assertIsNone(mesh_manager.add(height=1))
        self.assertIsNone(mesh_manager.add(radius=1))

        wrong_inputs = [
            -10,
            [1, 2],
            None,
            dict(),
            'a'
        ]

        for radius in wrong_inputs:
            with self.assertRaises(AssertionError):
                mesh_manager.add(type='cylinder', radius=radius, height=1)

        for height in wrong_inputs:
            with self.assertRaises(AssertionError):
                mesh_manager.add(type='cylinder', radius=1, height=height)

    def test_add_sphere_primitive(self):
        mesh_manager = MeshManager.get_instance()
        mesh_manager.reset()
        self.assertEqual(len(mesh_manager.tags), 0)

        tag = mesh_manager.add(
            tag='sphere_1', type='sphere', radius=1)
        self.assertEqual(tag, 'sphere_1')
        tag = mesh_manager.add(
            tag='sphere_2', type='sphere', radius=2)
        self.assertEqual(tag, 'sphere_2')

        # Add sphere with same size of sphere_1
        tag = mesh_manager.add(type='sphere', radius=1)
        self.assertEqual(tag, 'sphere_1')

        tag = mesh_manager.find_by_parameters(type='sphere', radius=2)
        self.assertEqual(tag, 'sphere_2')

        # Let the manager generate the tag
        self.assertIsNotNone(mesh_manager.add(
            type='sphere', radius=random.rand()))

    def test_add_wrong_sphere_input(self):
        mesh_manager = MeshManager.get_instance()
        mesh_manager.reset()
        self.assertEqual(len(mesh_manager.tags), 0)
        # Not specifying the type of the geometry
        self.assertIsNone(mesh_manager.add(radius=1))

        wrong_inputs = [
            -10,
            [1, 2],
            None,
            dict(),
            'a'
        ]

        for radius in wrong_inputs:
            with self.assertRaises(AssertionError):
                mesh_manager.add(type='sphere', radius=radius)

    def test_add_capsule_primitive(self):
        mesh_manager = MeshManager.get_instance()
        mesh_manager.reset()
        self.assertEqual(len(mesh_manager.tags), 0)
        tag = mesh_manager.add(
            tag='capsule_1', type='capsule', radius=1, height=1)
        self.assertEqual(tag, 'capsule_1')
        tag = mesh_manager.add(
            tag='capsule_2', type='capsule', radius=2, height=2)
        self.assertEqual(tag, 'capsule_2')

        # Add cylinder with same size of cyl_1
        tag = mesh_manager.add(type='capsule', radius=1, height=1)
        self.assertEqual(tag, 'capsule_1')

        tag = mesh_manager.find_by_parameters(
            type='capsule', radius=2, height=2)
        self.assertEqual(tag, 'capsule_2')

        # Let the manager generate the tag
        self.assertIsNotNone(mesh_manager.add(
            type='capsule',
            radius=random.rand(),
            height=random.rand()))

    def test_add_wrong_capsule_input(self):
        mesh_manager = MeshManager.get_instance()
        mesh_manager.reset()
        self.assertEqual(len(mesh_manager.tags), 0)
        # Not specifying the type of the geometry
        self.assertIsNone(mesh_manager.add(radius=1, height=1))
        # Missing parameters
        self.assertIsNone(mesh_manager.add(height=1))
        self.assertIsNone(mesh_manager.add(radius=1))

        wrong_inputs = [
            -10,
            [1, 2],
            None,
            dict(),
            'a'
        ]

        for radius in wrong_inputs:
            with self.assertRaises(AssertionError):
                mesh_manager.add(type='capsule', radius=radius, height=1)

        for height in wrong_inputs:
            with self.assertRaises(AssertionError):
                mesh_manager.add(type='capsule', radius=1, height=height)

    def test_add_mesh_file(self):
        mesh_manager = MeshManager.get_instance()
        mesh_manager.reset()
        self.assertEqual(len(mesh_manager.tags), 0)

        tag_1 = generate_random_string(5)
        tag = mesh_manager.add(
            tag=tag_1, filename=os.path.join(CWD, 'meshes', 'monkey.stl'))
        self.assertEqual(tag, tag_1)

        tag_2 = generate_random_string(5)
        tag = mesh_manager.add(
            tag=tag_2, filename=os.path.join(CWD, 'meshes', 'cube.stl'))
        self.assertEqual(tag, tag_2)

        self.assertEqual(
            mesh_manager.add(filename=os.path.join(CWD, 'meshes', 'cube.stl')),
            tag_2)


if __name__ == '__main__':
    unittest.main()
