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
import subprocess
import pytest
from pcg_gazebo.parsers.sdf import create_sdf_element
from pcg_gazebo.parsers import urdf2sdf, parse_urdf


def _to_print(obj):
    return subprocess.check_output(
        ['sdf2urdf', '--xml',
         obj.to_xml_as_str(), '--print']).decode('utf-8')


def _to_file(obj):
    pass


@pytest.mark.script_launch_mode('subprocess')
def test_xml_input_random(script_runner):
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
        'inertia'
    ]

    for sdf_name in sdf_elements:
        obj = create_sdf_element(sdf_name)
        assert obj is not None
        obj.random()
        output = script_runner.run(
            'sdf2urdf', '--xml', obj.to_xml_as_str(),
            '--print')
        assert output.success
        urdf = parse_urdf(output.stdout)
        assert urdf is not None

        response_sdf = urdf2sdf(urdf)
        assert obj == response_sdf

    # Test for dynamics
    obj = create_sdf_element('dynamics')
    assert obj is not None
    obj.reset(with_optional_elements=True)
    obj.damping.random()
    obj.friction.random()

    output = script_runner.run(
        'sdf2urdf', '--xml', obj.to_xml_as_str(),
        '--print')
    assert output.success
    urdf = parse_urdf(output.success)
    assert urdf is not None

    response_sdf = urdf2sdf(urdf)
    assert obj.damping == response_sdf.damping
    assert obj.friction == response_sdf.friction


@pytest.mark.script_launch_mode('subprocess')
def test_xml_input_visual_collision(script_runner):
    for sdf_tag in ['visual', 'collision']:
        obj = create_sdf_element(sdf_tag)
        assert obj is not None

        obj.pose = create_sdf_element('pose')
        obj.pose.random()
        obj.geometry = create_sdf_element('geometry')

        geometries = [
            'box',
            'cylinder',
            'sphere',
            'mesh'
        ]

        for geo_name in geometries:
            obj.geometry.reset(mode=geo_name)
            obj.geometry.random()

            output = script_runner.run(
                'sdf2urdf', '--xml', obj.to_xml_as_str(),
                '--print')
            assert output.success

            urdf = parse_urdf(output.stdout)
            assert urdf is not None

            response_sdf = urdf2sdf(urdf)

            assert obj == response_sdf
