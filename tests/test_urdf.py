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
from pcg_gazebo.parsers.urdf import create_urdf_element, \
    get_all_urdf_element_classes
from pcg_gazebo.parsers.gazebo import create_gazebo_element, \
    get_all_gazebo_element_classes


class TestURDFParser(unittest.TestCase):
    def test_reset_urdf(self):
        for c in get_all_urdf_element_classes():
            obj = create_urdf_element(c._NAME)
            self.assertIsNotNone(obj)
            if len(obj.modes):
                for tag in obj.modes:
                    obj.reset(mode=tag, with_optional_elements=True)
                    self.assertTrue(obj.is_valid())
            else:
                print(obj.xml_element_name)
                obj.reset(with_optional_elements=True)
                self.assertTrue(obj.is_valid())

    def test_reset_gazebo(self):
        for c in get_all_gazebo_element_classes():
            obj = create_gazebo_element(c._NAME)
            self.assertIsNotNone(obj)
            if len(obj.modes):
                for tag in obj.modes:
                    obj.reset(mode=tag, with_optional_elements=True)
                    self.assertTrue(obj.is_valid())
            else:
                print(obj.xml_element_name)
                obj.reset(with_optional_elements=True)
                self.assertTrue(obj.is_valid())


if __name__ == '__main__':
    unittest.main()
