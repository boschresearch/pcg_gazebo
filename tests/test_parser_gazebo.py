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
import os
import unittest
from pcg_gazebo.parsers import parse_urdf, urdf2sdf
from pcg_gazebo.parsers.gazebo import create_gazebo_element

GAZEBO_BASIC_OBJ_NAMES = [
    'kp',
    'kd',
    'material',
    'maxContacts',
    'maxVel',
    'minDepth',
    'mu1',
    'mu2',
    'provideFeedback',
    'selfCollide',
    'stopCfm',
    'stopErp'
]

VALID_BASIC_VALUES = dict(
    kp=[0.0, 0.1, 0.3, 0.8, 1, 10, 10000],
    kd=[0.0, 0.1, 0.3, 0.8, 1, 10, 10000],
    material=['Gazebo/Yellow', 'Gazebo/Blue'],
    maxContacts=[1, 2, 10, 10000],
    maxVel=[0.0, 0.1, 0.3, 0.8, 1, 10, 10000],
    minDepth=[0.0, 0.001, 0.1, 1.0],
    mu1=[0, 1, 0.3, 0.8],
    mu2=[0, 1, 0.3, 0.8],
    provideFeedback=[True, False],
    selfCollide=[True, False, 0, 1],
    stopCfm=[0, 1, 0.3, 0.8],
    stopErp=[0, 1, 0.3, 0.8]
)

VALID_OUTPUT_STRING = dict(
    kp=['0.0', '0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    kd=['0.0', '0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    material=['Gazebo/Yellow', 'Gazebo/Blue'],
    maxContacts=['1', '2', '10', '10000'],
    maxVel=['0.0', '0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    minDepth=['0.0', '0.001', '0.1', '1.0'],
    mu1=['0.0', '1.0', '0.3', '0.8'],
    mu2=['0.0', '1.0', '0.3', '0.8'],
    provideFeedback=['1', '0', '0', '1'],
    selfCollide=['1', '0', '0', '1'],
    stopCfm=['0.0', '1.0', '0.3', '0.8'],
    stopErp=['0.0', '1.0', '0.3', '0.8']
)

DEFAULT_SCALAR_VALUES = dict(
    kp=1e12,
    kd=1,
    material='none',
    maxContacts=20,
    maxVel=0.01,
    minDepth=0,
    mu1=1,
    mu2=1,
    provideFeedback=False,
    selfCollide=False,
    stopCfm=0,
    stopErp=0.2
)

INVALID_BASIC_VALUES = dict(
    kp=[-1, dict(), True, None, list(), 'a', [1, 2]],
    kd=[-1, dict(), True, None, list(), 'a', [1, 2]],
    material=[-1, dict(), True, None, list(), [1, 2]],
    maxContacts=[-1, dict(), True, None, list(), 'a', [1, 2]],
    maxVel=[-1, dict(), True, None, list(), 'a', [1, 2]],
    minDepth=[-1, dict(), True, None, list(), 'a', [1, 2]],
    mu1=[-1, dict(), True, None, list(), 'a', [1, 2]],
    mu2=[-1, dict(), True, None, list(), 'a', [1, 2]],
    provideFeedback=[-1, 2, dict(), None, list(), 'a', [1, 2]],
    selfCollide=[-1, 2, dict(), None, list(), 'a', [1, 2]],
    stopCfm=[-1, dict(), True, None, list(), 'a', [1, 2]],
    stopErp=[-1, dict(), True, None, list(), 'a', [1, 2]]
)

SDF_CONVERSION = dict(
    kp=dict(default=1e12),
    kd=dict(default=1),
    maxContacts=dict(default=20),
    maxVel=dict(default=0.01),
    minDepth=dict(default=0),
    mu1=dict(ode=1, bullet=1),
    mu2=dict(ode=1, bullet=1),
    provideFeedback=dict(default=False),
    selfCollide=dict(default=True),
    stopCfm=dict(default=0),
    stopErp=dict(default=0.2)
)

CUR_DIR = os.path.dirname(os.path.abspath(__file__))


def get_urdf_file(name):
    return os.path.join(CUR_DIR, 'urdf', '{}.urdf'.format(name))


class TestGazeboParser(unittest.TestCase):
    def test_basic_element_types(self):
        for tag in GAZEBO_BASIC_OBJ_NAMES:
            obj = create_gazebo_element(tag)

            self.assertIsNotNone(
                obj, '{} is invalid'.format(tag))
            self.assertEqual(obj.xml_element_name, tag,
                             '{} has invalid Gazebo block name'.format(tag))
            self.assertTrue(obj.has_value(),
                            '{} should store a values'.format(tag))

            # Testing default value
            self.assertEqual(obj.value, DEFAULT_SCALAR_VALUES[tag],
                             '{} has wrong default value, value={}'.format(
                             tag, obj.value))

            for value, value_str in zip(
                    VALID_BASIC_VALUES[tag],
                    VALID_OUTPUT_STRING[tag]):
                obj.value = value
                self.assertEqual(
                    obj.value, value,
                    'New value for {} was not set, new_value={}'.format(
                        tag, value))

                self.assertEqual(
                    obj.get_formatted_value_as_str(),
                    value_str,
                    'Wrong formatted output string for {}, expected={}, '
                    'returned={}'.format(
                        tag,
                        value_str,
                        obj.get_formatted_value_as_str()))

                # Test generation of XML block
                xml_element = obj.to_xml()
                self.assertEqual(
                    xml_element.tag,
                    tag,
                    'Invalid XML element tag for {}'.format(tag))
                self.assertEqual(
                    xml_element.text,
                    value_str,
                    'Invalid XML element tag for {}'.format(tag))

            for value in INVALID_BASIC_VALUES[tag]:
                with self.assertRaises(AssertionError):
                    obj.value = value

            if tag != 'material':
                for input_tag in SDF_CONVERSION[tag]:
                    obj.value = SDF_CONVERSION[tag][input_tag]
                    if input_tag == 'default':
                        sdf_obj = obj.to_sdf()
                    else:
                        sdf_obj = obj.to_sdf(input_tag)
                    self.assertIsNotNone(
                        sdf_obj,
                        '{} conversion to SDF returned an'
                        ' invalid object'.format(
                            tag))
                    self.assertEqual(
                        sdf_obj.value,
                        SDF_CONVERSION[tag][input_tag],
                        'Converted SDF object for {} has the wrong set '
                        'value'.format(tag))
            else:
                sdf_obj = obj.to_sdf()
                self.assertIsNotNone(sdf_obj)
                self.assertIsNotNone(sdf_obj.script)
                self.assertIsNotNone(sdf_obj.script.uris)
                self.assertIsNotNone(sdf_obj.script.name)

                self.assertEqual(len(sdf_obj.script.uris), 1)
                self.assertEqual(
                    sdf_obj.script.uris[0].value,
                    'file://media/materials/scripts/gazebo.material')
                self.assertEqual(sdf_obj.script.name.value, obj.value)

    def test_parse_gazebo_default_link(self):
        filename = get_urdf_file('gazebo_default_link')
        obj = parse_urdf(filename)

        elems = dict(
            mu1=1.0,
            mu2=1.0,
            kp=1000000.0,
            kd=100.0,
            minDepth=0.001,
            maxVel=1.0,
            selfCollide=False,
            maxContacts=15,
            material='Gazebo/Yellow'
        )

        for tag in elems:
            self.assertTrue(hasattr(obj, tag))
            self.assertEqual(getattr(obj, tag).value, elems[tag])

    def test_parse_gazebo_default_joint(self):
        filename = get_urdf_file('gazebo_default_joint')
        obj = parse_urdf(filename)

        self.assertTrue(obj.is_valid(), 'Gazebo block should be valid')
        self.assertEqual(
            obj.reference,
            'joint',
            'Wrong value for reference, received={}, expected={}'.format(
                obj.reference,
                'joint'))

        elems = dict(
            stopCfm=0.1,
            stopErp=0.3,
            provideFeedback=False
        )

        for tag in elems:
            self.assertTrue(hasattr(obj, tag))
            self.assertEqual(getattr(obj, tag).value, elems[tag])

    def test_parse_gazebo_imu_sdf_1_4(self):
        filename = get_urdf_file('gazebo_imu_1_4')
        obj = parse_urdf(filename)

        self.assertIsNotNone(obj, filename)
        self.assertTrue(
            obj.is_valid(), 'Gazebo block should be valid')
        self.assertIn(
            'sensor', obj.children, 'Sensor block was not loaded')
        self.assertEqual(
            len(obj.children['sensor']),
            1, 'One sensor element must should be in the children sensor list')

        # Checking general Gazebo sensor parameters
        sensor = obj.children['sensor'][0]
        self.assertEqual(
            sensor.name,
            'custom_imu',
            'Wrong value for name, received={}, expected={}'.format(
                sensor.name, 'custom_imu'))
        self.assertEqual(
            sensor.pose.value,
            [1, 2, 3, 4, 5, 6],
            'Wrong value for pose, received={}, expected={}'.format(
                sensor.pose.value, [1, 2, 3, 4, 5, 6]))
        self.assertEqual(
            sensor.visualize.value,
            1,
            'Wrong value for visualize flag, received={}, expected={}'.format(
                sensor.visualize.value, 1))
        self.assertEqual(
            sensor.topic.value,
            'sensor_topic',
            'Wrong value for topic name, received={}, expected={}'.format(
                sensor.topic.value, 1))
        self.assertEqual(
            sensor.update_rate.value,
            50, 'Wrong value for update rate, received={}, expected={}'.format(
                sensor.update_rate.value, 50))
        self.assertEqual(
            sensor.always_on.value,
            True,
            'Wrong value for always on flag, received={}, '
            'expected={}'.format(
                sensor.always_on.value,
                True))
        self.assertIsNotNone(sensor.imu, 'IMU elements was not parsed')

        # Checking IMU parameters
        imu = obj.children['sensor'][0].imu
        self.assertIsNotNone(
            imu.noise,
            'For SDF version 1.4, noise element should be valid')
        self.assertEqual(
            imu.noise.type.value,
            'gaussian',
            'Wrong value for type (SDF 1.4), received={}, '
            'expected={}'.format(
                imu.noise.type.value, True))
        self.assertEqual(
            imu.noise.accel.mean.value,
            0.1,
            'Wrong value for accel.mean value (SDF 1.4), '
            'received={}, expected={}'.format(
                imu.noise.accel.mean.value,
                0.1))
        self.assertEqual(
            imu.noise.accel.bias_mean.value,
            3,
            'Wrong value for accel.bias_mean value (SDF 1.4), '
            'received={}, expected={}'.format(
                imu.noise.accel.bias_mean.value,
                3))
        self.assertEqual(
            imu.noise.accel.stddev.value,
            0.01,
            'Wrong value for accel.stddev value (SDF 1.4), '
            'received={}, expected={}'.format(
                imu.noise.accel.stddev.value,
                0.01))
        self.assertEqual(
            imu.noise.accel.bias_stddev.value,
            3,
            'Wrong value for accel.bias_stddev value (SDF 1.4), '
            'received={}, expected={}'.format(
                imu.noise.accel.bias_stddev.value,
                3))
        self.assertEqual(
            imu.noise.rate.mean.value,
            0.1,
            'Wrong value for rate.mean value (SDF 1.4), '
            'received={}, expected={}'.format(
                imu.noise.rate.mean.value,
                0.1))
        self.assertEqual(
            imu.noise.rate.bias_mean.value,
            3,
            'Wrong value for rate.bias_mean value (SDF 1.4), '
            'received={}, expected={}'.format(
                imu.noise.rate.bias_mean.value,
                3))
        self.assertEqual(
            imu.noise.rate.stddev.value,
            0.01,
            'Wrong value for rate.stddev value (SDF 1.4), '
            'received={}, expected={}'.format(
                imu.noise.rate.stddev.value,
                0.01))
        self.assertEqual(
            imu.noise.rate.bias_stddev.value,
            3,
            'Wrong value for rate.bias_stddev value (SDF 1.4), '
            'received={}, expected={}'.format(
                imu.noise.rate.bias_stddev.value,
                3))

        # Checking plugin parameters
        self.assertIsNotNone(sensor.plugins, 'No plugin found for sensor')
        self.assertEqual(len(sensor.plugins), 1,
                         'There should be one plugin for sensor')

        plugin = sensor.plugins[0]
        print(plugin)
        self.assertEqual(
            plugin.name,
            'some_imu_plugin',
            'Wrong value for plugin.name value (SDF 1.4), '
            'received={}, expected={}'.format(
                plugin.name,
                'some_imu_plugin'))
        self.assertEqual(
            plugin.filename,
            'imu_ros_plugin.so',
            'Wrong value for plugin.filename value (SDF 1.4), '
            'received={}, expected={}'.format(
                plugin.filename,
                'imu_ros_plugin.so'))
        self.assertIn(
            'param', plugin.value,
            'Plugin parameter <param> not found')
        self.assertEqual(
            plugin.value['param'], 10,
            'Wrong value for plugin.param value (SDF 1.4), '
            'received={}, expected={}'.format(
                plugin.value['param'], 10))
        self.assertIn(
            'block', plugin.value,
            'Plugin parameter <block> not found')
        self.assertIn('subparam', plugin.value['block'],
                      'Plugin block parameter <subparam> not found')
        self.assertEqual(
            plugin.value['block']['subparam'], 1.245,
            'Wrong value for plugin.block.subparam value (SDF 1.4), '
            'received={}, expected={}'.format(
                plugin.value['block']['subparam'], 1.245))

    def test_parse_gazebo_imu_sdf_1_5(self):
        filename = get_urdf_file('gazebo_imu_1_5')
        obj = parse_urdf(filename)

        self.assertTrue(
            obj.is_valid(), 'Gazebo block should be valid')
        self.assertIn(
            'sensor', obj.children, 'Sensor block was not loaded')
        self.assertEqual(
            len(obj.children['sensor']),
            1, 'One sensor element must should be in the children sensor list')

        # Checking general Gazebo sensor parameters
        sensor = obj.children['sensor'][0]
        self.assertEqual(
            sensor.name,
            'custom_imu',
            'Wrong value for name, received={}, expected={}'.format(
                sensor.name, 'custom_imu'))
        self.assertEqual(
            sensor.pose.value,
            [1, 2, 3, 4, 5, 6],
            'Wrong value for pose, received={}, expected={}'.format(
                sensor.pose.value, [1, 2, 3, 4, 5, 6]))
        self.assertEqual(
            sensor.visualize.value,
            1,
            'Wrong value for visualize flag, received={}, expected={}'.format(
                sensor.visualize.value, 1))
        self.assertEqual(
            sensor.topic.value,
            'sensor_topic',
            'Wrong value for topic name, received={}, expected={}'.format(
                sensor.topic.value, 1))
        self.assertEqual(
            sensor.update_rate.value,
            50, 'Wrong value for update rate, received={}, expected={}'.format(
                sensor.update_rate.value, 50))
        self.assertEqual(
            sensor.always_on.value,
            True,
            'Wrong value for always on flag, received={}, expected={}'.format(
                sensor.always_on.value,
                True))
        self.assertIsNotNone(sensor.imu, 'IMU elements was not parsed')

        # Checking IMU parameters
        # Checking noise parameters
        imu = obj.children['sensor'][0].imu
        self.assertIsNotNone(
            imu.noise,
            'For SDF version 1.5, noise element should be valid')
        self.assertEqual(
            imu.noise.type.value,
            'gaussian',
            'Wrong value for type (SDF 1.5), received={}, '
            'expected={}'.format(
                imu.noise.type.value, True))
        self.assertEqual(
            imu.noise.accel.mean.value,
            0.2,
            'Wrong value for accel.mean value (SDF 1.5), '
            'received={}, expected={}'.format(
                imu.noise.accel.mean.value,
                0.2))
        self.assertEqual(
            imu.noise.accel.bias_mean.value,
            3,
            'Wrong value for accel.bias_mean value (SDF 1.5), '
            'received={}, expected={}'.format(
                imu.noise.accel.bias_mean.value,
                3))
        self.assertEqual(
            imu.noise.accel.stddev.value,
            0.02,
            'Wrong value for accel.stddev value (SDF 1.5), '
            'received={}, expected={}'.format(
                imu.noise.accel.stddev.value,
                0.02))
        self.assertEqual(
            imu.noise.accel.bias_stddev.value,
            3,
            'Wrong value for accel.bias_stddev value '
            '(SDF 1.5), received={}, expected={}'.format(
                imu.noise.accel.bias_stddev.value,
                3))
        self.assertEqual(
            imu.noise.rate.mean.value,
            0.2,
            'Wrong value for rate.mean value '
            '(SDF 1.5), received={}, expected={}'.format(
                imu.noise.rate.mean.value,
                0.2))
        self.assertEqual(
            imu.noise.rate.bias_mean.value,
            3,
            'Wrong value for rate.bias_mean value '
            '(SDF 1.5), received={}, expected={}'.format(
                imu.noise.rate.bias_mean.value,
                3))
        self.assertEqual(
            imu.noise.rate.stddev.value,
            0.02,
            'Wrong value for rate.stddev value '
            '(SDF 1.5), received={}, expected={}'.format(
                imu.noise.rate.stddev.value,
                0.02))
        self.assertEqual(
            imu.noise.rate.bias_stddev.value,
            3,
            'Wrong value for rate.bias_stddev value '
            '(SDF 1.5), received={}, expected={}'.format(
                imu.noise.rate.bias_stddev.value,
                3))

        # Checking noise parameters for angular velocity and linear
        # acceleration
        for imu_element in ['angular_velocity', 'linear_acceleration']:
            self.assertTrue(
                hasattr(imu, imu_element),
                'IMU has no element {}'.format(imu_element))
            for component in ['x', 'y', 'z']:
                self.assertTrue(
                    hasattr(getattr(imu, imu_element), component),
                    'imu.{} has no element {}'.format(imu_element, component))

                self.assertTrue(
                    hasattr(
                        getattr(
                            getattr(
                                imu,
                                imu_element),
                            component),
                        'noise'),
                    'imu.{}.{} has no element noise'.format(
                        imu_element,
                        component))

                noise = getattr(getattr(imu, imu_element), component).noise

                if component == 'x':
                    self.assertEqual(
                        noise.type.value,
                        'none',
                        'Wrong value for noise imu.{}.{}.noise.type, '
                        'received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.type.value,
                            'none'))
                elif component == 'y':
                    self.assertEqual(
                        noise.type.value,
                        'gaussian',
                        'Wrong value for noise imu.{}.{}.noise.type, '
                        'received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.type.value,
                            'gaussian'))
                elif component == 'z':
                    self.assertEqual(
                        noise.type.value,
                        'gaussian_quantized',
                        'Wrong value for noise imu.{}.{}.noise.type, '
                        'received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.type.value,
                            'gaussian_quantized'))

                self.assertEqual(
                    noise.mean.value,
                    0.2,
                    'Wrong value for noise imu.{}.{}.noise.mean, '
                    'received={}, expected={}'.format(
                        imu_element,
                        component,
                        noise.mean.value,
                        0.2))
                self.assertEqual(
                    noise.stddev.value,
                    0.02,
                    'Wrong value for noise imu.{}.{}.noise.stddev, '
                    'received={}, expected={}'.format(
                        imu_element,
                        component,
                        noise.stddev.value,
                        0.02))
                self.assertEqual(
                    noise.bias_mean.value,
                    4,
                    'Wrong value for noise imu.{}.{}.noise.bias_mean, '
                    'received={}, expected={}'.format(
                        imu_element,
                        component,
                        noise.bias_mean.value,
                        4))
                self.assertEqual(
                    noise.bias_stddev.value,
                    4,
                    'Wrong value for noise imu.{}.{}.noise.bias_stddev, '
                    'received={}, expected={}'.format(
                        imu_element,
                        component,
                        noise.bias_stddev.value,
                        4))
                self.assertEqual(
                    noise.precision.value,
                    0.5,
                    'Wrong value for noise imu.{}.{}.noise.precision, '
                    'received={}, expected={}'.format(
                        imu_element,
                        component,
                        noise.precision.value,
                        0.5))

        # Checking plugin parameters
        self.assertIsNotNone(sensor.plugins, 'No plugin found for sensor')
        self.assertEqual(len(sensor.plugins), 1,
                         'There should be one plugin for sensor')

        plugin = sensor.plugins[0]
        self.assertEqual(
            plugin.name,
            'some_imu_plugin',
            'Wrong value for plugin.name value (SDF 1.5), '
            'received={}, expected={}'.format(
                plugin.name,
                'some_imu_plugin'))
        self.assertEqual(
            plugin.filename,
            'imu_ros_plugin.so',
            'Wrong value for plugin.filename value (SDF 1.5), '
            'received={}, expected={}'.format(
                plugin.filename,
                'imu_ros_plugin.so'))
        self.assertIn(
            'param', plugin.value,
            'Plugin parameter <param> not found')
        self.assertEqual(
            plugin.value['param'], 10,
            'Wrong value for plugin.param value (SDF 1.5), '
            'received={}, expected={}'.format(
                plugin.value['param'], 10))
        self.assertIn(
            'block', plugin.value,
            'Plugin parameter <block> not found')
        self.assertIn('subparam', plugin.value['block'],
                      'Plugin block parameter <subparam> not found')
        self.assertEqual(
            plugin.value['block']['subparam'], 1.245,
            'Wrong value for plugin.block.subparam value (SDF 1.5), '
            'received={}, expected={}'.format(
                plugin.value['block']['subparam'], 1.245))

    def test_parse_gazebo_imu_sdf_1_6(self):
        filename = get_urdf_file('gazebo_imu_1_6')
        obj = parse_urdf(filename)

        self.assertTrue(
            obj.is_valid(), 'Gazebo block should be valid')
        self.assertIn(
            'sensor', obj.children, 'Sensor block was not loaded')
        self.assertEqual(
            len(obj.children['sensor']),
            1, 'One sensor element must should be in the children sensor list')

        # Checking general Gazebo sensor parameters
        sensor = obj.children['sensor'][0]
        self.assertEqual(
            sensor.name,
            'custom_imu',
            'Wrong value for name, received={}, expected={}'.format(
                sensor.name, 'custom_imu'))
        self.assertEqual(
            sensor.pose.value,
            [1, 2, 3, 4, 5, 6],
            'Wrong value for pose, received={}, expected={}'.format(
                sensor.pose.value, [1, 2, 3, 4, 5, 6]))
        self.assertEqual(
            sensor.visualize.value,
            1,
            'Wrong value for visualize flag, received={}, expected={}'.format(
                sensor.visualize.value, 1))
        self.assertEqual(
            sensor.topic.value,
            'sensor_topic',
            'Wrong value for topic name, received={}, expected={}'.format(
                sensor.topic.value, 1))
        self.assertEqual(
            sensor.update_rate.value,
            50, 'Wrong value for update rate, received={}, expected={}'.format(
                sensor.update_rate.value, 50))
        self.assertEqual(
            sensor.always_on.value,
            True,
            'Wrong value for always on flag, received={}, expected={}'.format(
                sensor.always_on.value,
                True))
        self.assertIsNotNone(sensor.imu, 'IMU elements was not parsed')

        # Checking IMU parameters
        imu = obj.children['sensor'][0].imu
        self.assertEqual(
            imu.topic.value, 'imu_topic',
            'Wrong value for imu.topic, received={}, expected={}'.format(
                imu.topic.value, 'imu_topic'))

        self.assertIsNotNone(
            imu.orientation_reference_frame.localization,
            'No localization element found for orientation_reference_frame')
        # Checking the IMU's orientation reference frame description
        self.assertEqual(
            imu.orientation_reference_frame.localization.value,
            'CUSTOM',
            'Wrong value for imu.orientation_reference_frame.localization,'
            ' received={}, expected={}'.format(
                imu.orientation_reference_frame.localization.value, 'CUSTOM'))

        # Checking noise parameters for angular velocity and linear
        # acceleration
        for imu_element in ['angular_velocity', 'linear_acceleration']:
            self.assertTrue(
                hasattr(imu, imu_element),
                'IMU has no element {}'.format(imu_element))
            for component in ['x', 'y', 'z']:
                self.assertTrue(
                    hasattr(getattr(imu, imu_element), component),
                    'imu.{} has no element {}'.format(imu_element, component))

                self.assertTrue(
                    hasattr(
                        getattr(
                            getattr(
                                imu,
                                imu_element),
                            component),
                        'noise'),
                    'imu.{}.{} has no element noise'.format(
                        imu_element,
                        component))

                noise = getattr(getattr(imu, imu_element), component).noise

                if component == 'x':
                    self.assertEqual(
                        noise.type.value,
                        'none',
                        'Wrong value for noise imu.{}.{}.noise.type, '
                        'received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.type.value,
                            'none'))
                elif component == 'y':
                    self.assertEqual(
                        noise.type.value,
                        'gaussian',
                        'Wrong value for noise imu.{}.{}.noise.type, '
                        'received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.type.value,
                            'gaussian'))
                elif component == 'z':
                    self.assertEqual(
                        noise.type.value,
                        'gaussian_quantized',
                        'Wrong value for noise imu.{}.{}.noise.type, '
                        'received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.type.value,
                            'gaussian_quantized'))

                self.assertEqual(
                    noise.mean.value,
                    0.2,
                    'Wrong value for noise imu.{}.{}.noise.mean, '
                    'received={}, expected={}'.format(
                        imu_element,
                        component,
                        noise.mean.value,
                        0.2))
                self.assertEqual(
                    noise.stddev.value,
                    0.02,
                    'Wrong value for noise imu.{}.{}.noise.stddev, '
                    'received={}, expected={}'.format(
                        imu_element,
                        component,
                        noise.stddev.value,
                        0.02))
                self.assertEqual(
                    noise.bias_mean.value,
                    4,
                    'Wrong value for noise imu.{}.{}.noise.bias_mean, '
                    'received={}, expected={}'.format(
                        imu_element,
                        component,
                        noise.bias_mean.value,
                        4))
                self.assertEqual(
                    noise.bias_stddev.value,
                    4,
                    'Wrong value for noise imu.{}.{}.noise.bias_stddev, '
                    'received={}, expected={}'.format(
                        imu_element,
                        component,
                        noise.bias_stddev.value,
                        4))
                self.assertEqual(
                    noise.precision.value,
                    0.5,
                    'Wrong value for noise imu.{}.{}.noise.precision, '
                    'received={}, expected={}'.format(
                        imu_element,
                        component,
                        noise.precision.value,
                        0.5))

        # Checking plugin parameters
        self.assertIsNotNone(
            sensor.plugins,
            'No plugin found for sensor')
        self.assertEqual(len(sensor.plugins), 1,
                         'There should be one plugin for sensor')

        plugin = sensor.plugins[0]
        self.assertEqual(
            plugin.name,
            'some_imu_plugin',
            'Wrong value for plugin.name value (SDF 1.6), '
            'received={}, expected={}'.format(
                plugin.name,
                'some_imu_plugin'))
        self.assertEqual(
            plugin.filename,
            'imu_ros_plugin.so',
            'Wrong value for plugin.filename value (SDF 1.6), '
            'received={}, expected={}'.format(
                plugin.filename,
                'imu_ros_plugin.so'))
        self.assertIn(
            'param', plugin.value,
            'Plugin parameter <param> not found')
        self.assertEqual(
            plugin.value['param'], 10,
            'Wrong value for plugin.param value (SDF 1.6), '
            'received={}, expected={}'.format(
                plugin.value['param'], 10))
        self.assertIn(
            'block', plugin.value,
            'Plugin parameter <block> not found')
        self.assertIn('subparam', plugin.value['block'],
                      'Plugin block parameter <subparam> not found')
        self.assertEqual(
            plugin.value['block']['subparam'], 1.245,
            'Wrong value for plugin.block.subparam value'
            ' (SDF 1.6), received={}, expected={}'.format(
                plugin.value['block']['subparam'], 1.245))

    def test_gazebo_material_single_link_single_visual(self):
        filename = get_urdf_file(
            'gazebo_material_single_link_single_visual')
        obj = parse_urdf(filename)

        self.assertIsNotNone(obj)
        self.assertTrue(obj.is_valid())

        self.assertEqual(obj.xml_element_name, 'robot')

        sdf = urdf2sdf(obj)

        self.assertIsNotNone(sdf)
        self.assertIsNotNone(sdf.links)
        self.assertEqual(len(sdf.links), 1)
        self.assertIsNotNone(sdf.links[0].visuals)

        self.assertEqual(
            sdf.links[0].visuals[0].material.script.uris[0].value,
            'file://media/materials/scripts/gazebo.material')
        self.assertEqual(
            sdf.links[0].visuals[0].material.script.name.value,
            'Gazebo/Blue')

    def test_gazebo_material_single_link_mult_visual(self):
        filename = get_urdf_file(
            'gazebo_material_single_link_mult_visual')
        obj = parse_urdf(filename)

        self.assertIsNotNone(obj)
        self.assertTrue(obj.is_valid())

        self.assertEqual(obj.xml_element_name, 'robot')

        sdf = urdf2sdf(obj)

        self.assertIsNotNone(sdf)
        self.assertIsNotNone(sdf.links)
        self.assertEqual(len(sdf.links), 1)
        self.assertIsNotNone(sdf.links[0].visuals)
        self.assertEqual(len(sdf.links[0].visuals), 3)

        for visual in sdf.links[0].visuals:
            self.assertEqual(
                visual.material.script.uris[0].value,
                'file://media/materials/scripts/gazebo.material')
            self.assertEqual(
                visual.material.script.name.value,
                'Gazebo/Blue')


if __name__ == '__main__':
    unittest.main()
