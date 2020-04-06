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
from pcg_gazebo import random
from pcg_gazebo.simulation.properties import Inertial


CORRECT_ROS_NAMESPACE = '/pcg_test'


def get_solid_sphere_inertia(mass, radius):
    fac = 2. / 5
    return dict(
        mass=mass,
        ixx=fac * mass * radius**2,
        iyy=fac * mass * radius**2,
        izz=fac * mass * radius**2,
        ixy=0,
        ixz=0,
        iyz=0
    )


def get_hollow_sphere_inertia(mass, radius):
    fac = 2. / 3
    return dict(
        mass=mass,
        ixx=fac * mass * radius**2,
        iyy=fac * mass * radius**2,
        izz=fac * mass * radius**2,
        ixy=0,
        ixz=0,
        iyz=0
    )


def get_ellipsoid_inertia(mass, axis_length_x, axis_length_y, axis_length_z):
    fac = 1. / 5
    return dict(
        mass=mass,
        ixx=fac * mass * (axis_length_y**2 + axis_length_z**2),
        iyy=fac * mass * (axis_length_x**2 + axis_length_z**2),
        izz=fac * mass * (axis_length_x**2 + axis_length_y**2),
        ixy=0,
        ixz=0,
        iyz=0
    )


def get_cuboid_inertia(mass, length_x, length_y, length_z):
    fac = 1. / 12
    return dict(
        mass=mass,
        ixx=fac * mass * (length_y**2 + length_z**2),
        iyy=fac * mass * (length_x**2 + length_z**2),
        izz=fac * mass * (length_x**2 + length_y**2),
        ixy=0,
        ixz=0,
        iyz=0
    )


def get_centered_rod_inertia(mass, length, axis):
    fac = 1. / 12
    return dict(
        mass=mass,
        ixx=axis[0] * fac * mass * length**2,
        iyy=axis[1] * fac * mass * length**2,
        izz=axis[2] * fac * mass * length**2,
        ixy=0,
        ixz=0,
        iyz=0
    )


class TestPCGGeneratorInertial(unittest.TestCase):
    def test_default_values(self):
        inertia = Inertial()
        self.assertEqual(inertia.ixx, 0)
        self.assertEqual(inertia.ixy, 0)
        self.assertEqual(inertia.ixz, 0)
        self.assertEqual(inertia.iyy, 0)
        self.assertEqual(inertia.iyz, 0)
        self.assertEqual(inertia.izz, 0)

    def test_solid_sphere_inertia(self):
        # Test invalid inputs
        inputs = [
            [-1, 1],
            [1, -1],
            [0, 1],
            [1, 0]
        ]
        for test_input in inputs:
            with self.assertRaises(AssertionError):
                Inertial.create_hollow_sphere_inertia(*test_input)

        mass = random.rand()
        radius = random.rand()
        ref_inertia = get_solid_sphere_inertia(mass, radius)

        inertia = Inertial.create_solid_sphere_inertia(mass, radius)
        for tag in ref_inertia:
            self.assertEqual(
                ref_inertia[tag],
                getattr(inertia, tag),
                'Element {} set with wrong value, '
                'correct={}, value={}'.format(
                    tag, ref_inertia[tag], getattr(
                        inertia, tag)))

    def test_hollow_sphere_inertia(self):
        # Test invalid inputs
        inputs = [
            [-1, 1],
            [1, -1],
            [0, 1],
            [1, 0]
        ]
        for test_input in inputs:
            with self.assertRaises(AssertionError):
                Inertial.create_hollow_sphere_inertia(*test_input)

        mass = random.rand()
        radius = random.rand()
        ref_inertia = get_hollow_sphere_inertia(mass, radius)

        inertia = Inertial.create_hollow_sphere_inertia(mass, radius)
        for tag in ref_inertia:
            self.assertEqual(
                ref_inertia[tag],
                getattr(inertia, tag),
                'Element {} set with wrong value, '
                'correct={}, value={}'.format(
                    tag, ref_inertia[tag], getattr(
                        inertia, tag)))

    def test_ellipsoid_inertia(self):
        # Test invalid inputs
        inputs = [
            [0, 0, 0, 0],
            [0, 1, 1, 1],
            [1, 0, 1, 1],
            [1, 1, 0, 1],
            [1, 1, 1, 0]
        ]
        for test_input in inputs:
            with self.assertRaises(AssertionError):
                Inertial.create_ellipsoid_inertia(*test_input)

        mass = random.rand()
        axis_length_x = random.rand()
        axis_length_y = random.rand()
        axis_length_z = random.rand()

        ref_inertia = get_ellipsoid_inertia(
            mass, axis_length_x, axis_length_y, axis_length_z)

        inertia = Inertial.create_ellipsoid_inertia(
            mass, axis_length_x, axis_length_y, axis_length_z)

        for tag in ref_inertia:
            self.assertEqual(
                ref_inertia[tag],
                getattr(inertia, tag),
                'Element {} set with wrong value, '
                'correct={}, value={}'.format(
                    tag, ref_inertia[tag], getattr(
                        inertia, tag)))

    def test_cuboid_inertia(self):
        # Test invalid inputs
        inputs = [
            [0, 0, 0, 0],
            [0, 1, 1, 1],
            [1, 0, 1, 1],
            [1, 1, 0, 1],
            [1, 1, 1, 0]
        ]
        for test_input in inputs:
            with self.assertRaises(AssertionError):
                Inertial.create_cuboid_inertia(*test_input)

        mass = random.rand()
        length_x = random.rand()
        length_y = random.rand()
        length_z = random.rand()

        ref_inertia = get_cuboid_inertia(
            mass, length_x, length_y, length_z)

        inertia = Inertial.create_cuboid_inertia(
            mass, length_x, length_y, length_z)

        for tag in ref_inertia:
            self.assertEqual(
                ref_inertia[tag],
                getattr(inertia, tag),
                'Element {} set with wrong value, '
                'correct={}, value={}'.format(
                    tag, ref_inertia[tag], getattr(
                        inertia, tag)))

    def test_centered_rod_inerita(self):
        # Test invalid inputs
        inputs = [
            [0, 0, []],
            [1, 1, None],
            [1, 1, [1, 1]],
            [1, 1, [1, 1, 1]],
        ]
        for test_input in inputs:
            with self.assertRaises(AssertionError):
                Inertial.create_centered_rod_inertia(*test_input)

        inputs = [
            [random.rand(), random.rand(), [1, 0, 0]],
            [random.rand(), random.rand(), [0, 1, 0]],
            [random.rand(), random.rand(), [0, 0, 1]]
        ]

        for test_input in inputs:
            ref_inertia = get_centered_rod_inertia(*test_input)

            inertia = Inertial.create_centered_rod_inertia(*test_input)

            for tag in ref_inertia:
                self.assertEqual(
                    ref_inertia[tag],
                    getattr(inertia, tag),
                    'Element {} set with wrong '
                    'value, correct={}, value={}'.format(
                        tag, ref_inertia[tag], getattr(
                            inertia, tag)))


if __name__ == '__main__':
    unittest.main()
