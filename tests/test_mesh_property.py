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
import os
import sys
import unittest
import numpy as np
from pcg_gazebo.simulation.properties import Mesh


CUR_DIR = os.path.dirname(os.path.abspath(__file__))
CUBE_FILENAME_PREFIX = os.path.join(CUR_DIR, 'meshes', 'cube')
MONKEY_FILENAME_PREFIX = os.path.join(CUR_DIR, 'meshes', 'monkey')
MONKEY_X_RANGE = 1.70312
MONKEY_Y_RANGE = 2.73438
MONKEY_Z_RANGE = 1.96875
MONKEY_OFFSET = 1


class TestSimulationObjectProperties(unittest.TestCase):
    def test_cube_mesh_stl(self):
        # Try and load the wrong format
        with self.assertRaises(AssertionError):
            mesh = Mesh(filename=CUBE_FILENAME_PREFIX + '.abc', load_mesh=True)

        mesh = Mesh(filename=CUBE_FILENAME_PREFIX + '.stl', load_mesh=True)
        self.assertIsNotNone(mesh.mesh, 'Mesh object was not loaded')
        self.assertIsNotNone(mesh.bounds, 'Mesh bounds were not computed')

        bounds = dict(
            lower_x=-1,
            upper_x=1,
            lower_y=-1,
            upper_y=1,
            lower_z=-1,
            upper_z=1
        )

        # Test bound values
        for tag in bounds:
            self.assertTrue(
                np.isclose(
                    mesh.bounds[tag],
                    bounds[tag],
                    1e-4),
                msg='Bound {} is incorrect, expected={}, retrieved={}'.format(
                    tag,
                    bounds[tag],
                    mesh.bounds[tag]))

    def test_monkey_mesh_stl(self):
        # Try and load the wrong format
        with self.assertRaises(AssertionError):
            mesh = Mesh(
                filename=MONKEY_FILENAME_PREFIX +
                '.abc',
                load_mesh=True)

        mesh = Mesh(filename=MONKEY_FILENAME_PREFIX + '.stl', load_mesh=True)
        self.assertIsNotNone(mesh.mesh, 'Mesh object was not loaded')
        self.assertIsNotNone(mesh.bounds, 'Mesh bounds were not computed')

        bounds = dict(
            lower_x=-1 * MONKEY_X_RANGE / 2,
            upper_x=MONKEY_X_RANGE / 2,
            lower_y=-1 * MONKEY_Y_RANGE / 2,
            upper_y=MONKEY_Y_RANGE / 2,
            lower_z=-1 * MONKEY_Z_RANGE / 2,
            upper_z=MONKEY_Z_RANGE / 2
        )

        # Test bound values
        for tag in bounds:
            self.assertTrue(
                np.isclose(
                    mesh.bounds[tag],
                    bounds[tag],
                    1e-4),
                msg='Bound {} is incorrect, expected={}, retrieved={}'.format(
                    tag,
                    bounds[tag],
                    mesh.bounds[tag]))

        # Load monkey with offset
        mesh = Mesh(
            filename=MONKEY_FILENAME_PREFIX +
            '_offset.stl',
            load_mesh=True)
        self.assertIsNotNone(mesh.mesh, 'Mesh {} object was not loaded'.format(
            MONKEY_FILENAME_PREFIX + '_offset.stl'))
        self.assertIsNotNone(
            mesh.bounds,
            'Mesh bounds for {} were not computed'.format(
                MONKEY_FILENAME_PREFIX +
                '_offset.stl'))

        # Test bound values
        for tag in bounds:
            self.assertTrue(
                np.isclose(
                    mesh.bounds[tag],
                    bounds[tag] +
                    MONKEY_OFFSET,
                    1e-4),
                msg='Bound {} is incorrect, expected={}, retrieved={}'.format(
                    tag,
                    bounds[tag] +
                    MONKEY_OFFSET,
                    mesh.bounds[tag]))

    def test_cube_mesh_collada(self):
        if sys.version_info[0] == 2:
            return
        with self.assertRaises(AssertionError):
            mesh = Mesh(filename=CUBE_FILENAME_PREFIX + '.abc', load_mesh=True)

        mesh = Mesh(filename=CUBE_FILENAME_PREFIX + '.dae', load_mesh=True)
        self.assertIsNotNone(mesh.mesh, 'Mesh object was not loaded')
        self.assertIsNotNone(mesh.bounds, 'Mesh bounds were not computed')


if __name__ == '__main__':
    unittest.main()
