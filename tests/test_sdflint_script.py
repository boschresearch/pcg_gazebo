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
import pytest
from pcg_gazebo.parsers.sdf import create_sdf_element


@pytest.mark.script_launch_mode('subprocess')
def test_random_sdf(script_runner):
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
        output = script_runner.run(
            'pcg-sdflint', '--xml',
            obj.to_xml_as_str())

        assert output.success


# @pytest.mark.script_launch_mode('subprocess')
# def test_run_linter_on_test_files(script_runner):
#     test_folder = os.path.join(
#         os.path.dirname(os.path.abspath(__file__)),
#         'sdf')
#     for filename in os.listdir(test_folder):
#         if '.sdf' not in filename:
#             continue
#         output = script_runner.run(
#             'sdflint', '--filename',
#             os.path.join(test_folder, filename))

#         assert output.success
