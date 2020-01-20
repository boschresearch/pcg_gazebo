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
import unittest
from pcg_gazebo.utils import load_yaml

CWD = os.path.dirname(os.path.abspath(__file__))


class TestLoadYAML(unittest.TestCase):
    def test_load_wrong_extension(self):
        with self.assertRaises(AssertionError):
            load_yaml(os.path.join(CWD, 'yaml', 'wrong_extension.ym'))

    def test_load_dummy_with_include(self):
        filename = os.path.join(CWD, 'yaml', 'dummy.yaml')
        data = load_yaml(filename)

        for tag in ['a', 'b', 'c']:
            self.assertIn(tag, data)

        self.assertEqual(data['a'], 1)
        self.assertEqual(data['b'], 2)
        self.assertIsInstance(data['c'], dict)
        self.assertEqual(data['c']['test'], 'a test')

    # TODO: Add helper function to dynamically create dummy catkin packages
    # def test_find_ros_package(self):
    #     filename = os.path.join(CWD, 'yaml', 'include_ros_package.yaml')
    #     data = load_yaml(filename)

    #     self.assertIn('a', data)
    #     self.assertIn('test', data['a'])

    def test_include_wrong_ros_package(self):
        filename = os.path.join(CWD, 'yaml', 'include_wrong_ros_package.yaml')
        with self.assertRaises(AssertionError):
            load_yaml(filename)

    def test_load_from_string(self):
        pass


if __name__ == '__main__':
    unittest.main()
