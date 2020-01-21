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
from pcg_gazebo.simulation.properties import Pose
import numpy as np


class TestPose(unittest.TestCase):
    def test_add_positions(self):
        for _ in range(10):
            p1 = Pose.random_position()
            p2 = Pose.random_position()

            p_result = p1 + p2
            self.assertEqual(
                np.sum(p1.position + p2.position - p_result.position),
                0)

            self.assertEqual(p_result.x, p1.x + p2.x)
            self.assertEqual(p_result.y, p1.y + p2.y)
            self.assertEqual(p_result.z, p1.z + p2.z)

    def test_add_quaternion(self):
        p_ref = Pose()

        for _ in range(10):
            p_target = Pose.random_orientation()

            p_result = p_ref + p_target
            self.assertEqual(np.sum(p_result.quat - p_target.quat), 0)

    def test_euler_transformation(self):
        for i in range(3):
            vec = np.zeros(3)
            vec[i] = 2 * np.pi * np.random.random()

            p = Pose(rot=Pose.rpy2quat(*vec))

            diff = Pose.get_transform(p.quat, Pose.rpy2quat(*vec))
            self.assertTrue(np.isclose(diff[0], 0))
            self.assertTrue(np.isclose(diff[1], 0))
            self.assertTrue(np.isclose(diff[2], 0))
            self.assertTrue(np.isclose(diff[3], 1))

    def test_to_sdf(self):
        for _ in range(10):
            p = Pose.random()
            sdf = p.to_sdf()
            self.assertEqual(sdf.value, p.position.tolist() + p.rpy)

    def test_to_urdf(self):
        for _ in range(10):
            p = Pose.random()
            urdf = p.to_urdf()
            self.assertTrue(
                np.isclose(np.sum(np.array(urdf.xyz) - p.position), 0))
            self.assertTrue(
                np.isclose(np.sum(np.array(urdf.rpy) - np.array(p.rpy)), 0))


if __name__ == '__main__':
    unittest.main()
