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
from __future__ import print_function
# import os
import unittest
import subprocess
from pcg_gazebo.parsers.sdf import create_sdf_element


class TestSDFLintScript(unittest.TestCase):
    def test_random_sdf(self):
        sdf_elements = [
            'mass',
            'child',
            'parent',
            'pose',
            'box',
            'cylinder',
            'sphere',
            'mesh',
            'limit',
            'inertial',
            'inertia',
            'joint',
            'link',
            'model',
            'visual',
            'collision',
            'image'
        ]

        for sdf_name in sdf_elements:
            obj = create_sdf_element(sdf_name)
            obj.random()
            subprocess.check_output(
                ['pcg-sdflint', '--xml',
                 obj.to_xml_as_str()])


if __name__ == '__main__':
    unittest.main()
