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
import unittest
import subprocess
from pcg_gazebo.parsers.urdf import create_urdf_element
from pcg_gazebo.parsers import sdf2urdf, parse_sdf


class TestURDF2SDFScript(unittest.TestCase):
    def test_xml_input_random(self):
        urdf_elements = [
            'mass',
            'child',
            'parent',
            'origin',
            'box',
            'cylinder',
            'sphere',
            'mesh',
            'limit',
            'inertial',
            'inertia',
            'dynamics'
        ]

        for urdf_name in urdf_elements:
            obj = create_urdf_element(urdf_name)
            assert obj is not None
            obj.random()
            output = subprocess.check_output(
                ['pcg-urdf2sdf', '--xml', obj.to_xml_as_str(),
                 '--print'])

            urdf = parse_sdf(output.decode('utf-8'))
            assert urdf is not None

            response_urdf = sdf2urdf(urdf)
            assert obj == response_urdf

    def test_xml_input_visual_collision(self):
        for urdf_tag in ['visual', 'collision']:
            obj = create_urdf_element(urdf_tag)
            assert obj is not None

            obj.origin = create_urdf_element('origin')
            obj.origin.random()
            obj.geometry = create_urdf_element('geometry')

            geometries = [
                'box',
                'cylinder',
                'sphere',
                'mesh'
            ]

            for geo_name in geometries:
                obj.geometry.reset(mode=geo_name)
                obj.geometry.random()

                output = subprocess.check_output(
                    ['pcg-urdf2sdf', '--xml', obj.to_xml_as_str(),
                     '--print'])
                urdf = parse_sdf(output.decode('utf-8'))
                assert urdf is not None

                response_urdf = sdf2urdf(urdf)

                assert obj == response_urdf


if __name__ == '__main__':
    unittest.main()
