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
import os
from pcg_gazebo import Path

CWD = os.path.dirname(os.path.abspath(__file__))


class TestResolvePaths(unittest.TestCase):
    def test_valid_file_paths(self):
        original_uri = os.path.join(
            CWD, 'meshes', 'cube.dae')

        self.assertTrue(os.path.isfile(original_uri))

        uris = [
            'file://' + original_uri,
            # 'package://pcg_libraries/test/meshes/cube.dae',
            # '$(find pcg_libraries)/test/meshes/cube.dae'
        ]

        for uri in uris:
            p = Path(uri)
            self.assertEqual(uri, p.original_uri)
            self.assertEqual(original_uri, p.absolute_uri)

    def test_gazebo_model_path(self):
        original_uri = 'model://test_joint_fixed/model.sdf'
        absolute_uri = os.path.join(
            CWD,
            'gazebo_models',
            'test_joint_fixed',
            'model.sdf')
        self.assertTrue(os.path.isfile(absolute_uri))
        p = Path(original_uri)

        self.assertEqual(p.original_uri, original_uri)
        self.assertEqual(p.absolute_uri, absolute_uri)
        self.assertEqual(p.model_uri, original_uri)
        self.assertEqual(p.file_uri, 'file://' + absolute_uri)
        # self.assertEqual(p.ros_package, 'pcg_libraries')
        self.assertEqual(p.gazebo_model, 'test_joint_fixed')

    def test_file_uri(self):
        absolute_uri = os.path.join(
            CWD, 'meshes', 'cube.dae')

        original_uri = 'file://' + absolute_uri
        self.assertTrue(os.path.isfile(absolute_uri))
        p = Path(original_uri)

        self.assertEqual(p.original_uri, original_uri)
        self.assertEqual(p.absolute_uri, absolute_uri)
        self.assertEqual(p.file_uri, original_uri)
        # self.assertEqual(p.ros_package, 'pcg_libraries')
        self.assertIsNone(p.gazebo_model)

    # def test_ros_package_uri(self):
    #     absolute_uri = os.path.join(
    #         CWD, 'meshes', 'cube.dae')
    #     original_uri = '$(find pcg_libraries)/test/meshes/cube.dae'
    #     self.assertTrue(os.path.isfile(absolute_uri))
    #     p = Path(original_uri)

    #     self.assertEqual(p.original_uri, original_uri)
    #     self.assertEqual(p.absolute_uri, absolute_uri)
    #     self.assertEqual(p.file_uri, 'file://' + absolute_uri)
    #     self.assertEqual(p.ros_package, 'pcg_libraries')
    #     self.assertIsNone(p.gazebo_model)

    # def test_package_uri(self):
    #     absolute_uri = os.path.join(
    #         CWD, 'meshes', 'cube.dae')
    #     original_uri = 'package://pcg_libraries/test/meshes/cube.dae'
    #     self.assertTrue(os.path.isfile(absolute_uri))
    #     p = Path(original_uri)

    #     self.assertEqual(p.original_uri, original_uri)
    #     self.assertEqual(p.absolute_uri, absolute_uri)
    #     self.assertEqual(p.file_uri, 'file://' + absolute_uri)
    #     self.assertEqual(p.ros_package, 'pcg_libraries')
    #     self.assertIsNone(p.gazebo_model)

    def test_invalid_uris(self):
        invalid_uri_types = [
            10,
            None,
            dict(),
            list()
        ]

        for uri in invalid_uri_types:
            with self.assertRaises(AssertionError):
                Path(uri)

        invalid_path_names = [
            'abc',
            'some/path',
            os.path.join(
                CWD, 'meshes', 'cube.dae.wrong')
        ]

        for uri in invalid_path_names:
            p = Path(uri)
            self.assertFalse(p.is_valid)


if __name__ == '__main__':
    unittest.main()
