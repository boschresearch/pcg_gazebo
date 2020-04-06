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
import unittest
import numpy as np
from pcg_gazebo.generators import patterns


class TestPatterns(unittest.TestCase):
    def test_circular_step(self):
        poses = patterns.circular(
            radius=10,
            max_theta=2 * np.pi,
            step_theta=2 * np.pi / 4,
            step_radius=1)
        self.assertIsNotNone(poses)

        x = [pose.x for pose in poses]
        y = [pose.y for pose in poses]

        self.assertEqual(np.min(x), -10)
        self.assertEqual(np.min(y), -10)

        self.assertEqual(np.max(x), 10)
        self.assertEqual(np.max(y), 10)

    def test_circular_n_elems(self):
        poses = patterns.circular(
            radius=10,
            max_theta=2 * np.pi,
            n_theta=4,
            n_radius=10)
        self.assertIsNotNone(poses)

        x = [pose.x for pose in poses]
        y = [pose.y for pose in poses]

        self.assertEqual(np.min(x), -10)
        self.assertEqual(np.min(y), -10)

        self.assertEqual(np.max(x), 10)
        self.assertEqual(np.max(y), 10)

    def test_rectangular_step(self):
        # Default rectangular pattern
        poses = patterns.rectangular(
            x_length=10,
            y_length=10,
            step_x=1,
            step_y=1,
            center=False)
        self.assertIsNotNone(poses)
        self.assertEqual(len(poses), 11 * 11)
        x = [pose.x for pose in poses]
        y = [pose.y for pose in poses]

        self.assertEqual(np.min(x), 0)
        self.assertEqual(np.min(y), 0)

        self.assertEqual(np.max(x), 10)
        self.assertEqual(np.max(y), 10)

        # Center the pattern
        poses = patterns.rectangular(
            x_length=10,
            y_length=10,
            step_x=1,
            step_y=1,
            center=True)
        self.assertIsNotNone(poses)
        self.assertEqual(len(poses), 11 * 11)
        x = [pose.x for pose in poses]
        y = [pose.y for pose in poses]

        self.assertEqual(np.min(x), -5)
        self.assertEqual(np.min(y), -5)

        self.assertEqual(np.max(x), 5)
        self.assertEqual(np.max(y), 5)

    def test_rectangular_n_elems(self):
        poses = patterns.rectangular(
            x_length=10,
            y_length=10,
            n_x=10,
            n_y=10,
            center=False)
        self.assertIsNotNone(poses)
        self.assertEqual(len(poses), 10 * 10)

        x = [pose.x for pose in poses]
        y = [pose.y for pose in poses]

        self.assertEqual(np.min(x), 0)
        self.assertEqual(np.min(y), 0)

        self.assertEqual(np.max(x), 10)
        self.assertEqual(np.max(y), 10)

        poses = patterns.rectangular(
            x_length=10,
            y_length=10,
            n_x=10,
            n_y=10,
            center=True)
        self.assertIsNotNone(poses)
        self.assertEqual(len(poses), 10 * 10)

        x = [pose.x for pose in poses]
        y = [pose.y for pose in poses]

        self.assertEqual(np.min(x), -5)
        self.assertEqual(np.min(y), -5)

        self.assertEqual(np.max(x), 5)
        self.assertEqual(np.max(y), 5)

    def test_cuboid_step(self):
        # Default rectangular pattern
        poses = patterns.cuboid(
            x_length=10,
            y_length=10,
            z_length=10,
            step_x=1,
            step_y=1,
            step_z=1,
            center=False)
        self.assertIsNotNone(poses)
        self.assertEqual(len(poses), 11 * 11 * 11)
        x = [pose.x for pose in poses]
        y = [pose.y for pose in poses]
        z = [pose.z for pose in poses]

        self.assertEqual(np.min(x), 0)
        self.assertEqual(np.min(y), 0)
        self.assertEqual(np.min(z), 0)

        self.assertEqual(np.max(x), 10)
        self.assertEqual(np.max(y), 10)
        self.assertEqual(np.max(z), 10)

        # Center the pattern
        poses = patterns.cuboid(
            x_length=10,
            y_length=10,
            z_length=10,
            step_x=1,
            step_y=1,
            step_z=1,
            center=True)
        self.assertIsNotNone(poses)
        self.assertEqual(len(poses), 11 * 11 * 11)
        x = [pose.x for pose in poses]
        y = [pose.y for pose in poses]
        z = [pose.z for pose in poses]

        self.assertEqual(np.min(x), -5)
        self.assertEqual(np.min(y), -5)
        self.assertEqual(np.min(z), -5)

        self.assertEqual(np.max(x), 5)
        self.assertEqual(np.max(y), 5)
        self.assertEqual(np.max(z), 5)

    def test_cuboid_n_elems(self):
        poses = patterns.cuboid(
            x_length=10,
            y_length=10,
            z_length=10,
            n_x=10,
            n_y=10,
            n_z=10,
            center=False)
        self.assertIsNotNone(poses)
        self.assertEqual(len(poses), 10 * 10 * 10)

        x = [pose.x for pose in poses]
        y = [pose.y for pose in poses]
        z = [pose.z for pose in poses]

        self.assertEqual(np.min(x), 0)
        self.assertEqual(np.min(y), 0)
        self.assertEqual(np.min(z), 0)

        self.assertEqual(np.max(x), 10)
        self.assertEqual(np.max(y), 10)
        self.assertEqual(np.max(z), 10)

        poses = patterns.cuboid(
            x_length=10,
            y_length=10,
            z_length=10,
            n_x=10,
            n_y=10,
            n_z=10,
            center=True)
        self.assertIsNotNone(poses)
        self.assertEqual(len(poses), 10 * 10 * 10)

        x = [pose.x for pose in poses]
        y = [pose.y for pose in poses]
        z = [pose.z for pose in poses]

        self.assertEqual(np.min(x), -5)
        self.assertEqual(np.min(y), -5)
        self.assertEqual(np.min(z), -5)

        self.assertEqual(np.max(x), 5)
        self.assertEqual(np.max(y), 5)
        self.assertEqual(np.max(z), 5)


if __name__ == '__main__':
    unittest.main()
