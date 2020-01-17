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
import sys
import unittest
import rospkg
from pcg_gazebo.parsers import parse_urdf
from pcg_gazebo.parsers.urdf import create_urdf_element

CUR_DIR = os.path.dirname(os.path.abspath(__file__))

URDF_BASIC_OBJ_NAMES = [
    'mu1',
    'mu2',
    'kp',
    'kd',
    'maxContacts',
    'minDepth',
    'maxVel',
    'selfCollide',
    'stopCfm',
    'stopErp'
]

VALID_BASIC_VALUES = dict(
    mu1=[0, 1, 0.3, 0.8],
    mu2=[0, 1, 0.3, 0.8],
    kp=[0.0, 0.1, 0.3, 0.8, 1, 10, 10000],
    kd=[0.0, 0.1, 0.3, 0.8, 1, 10, 10000],
    maxContacts=[1, 2, 10, 10000],
    minDepth=[0.0, 0.001, 0.1, 1.0],
    maxVel=[0.0, 0.1, 0.3, 0.8, 1, 10, 10000],
    selfCollide=[True, False, 0, 1],
    stopCfm=[0, 1, 0.3, 0.8],
    stopErp=[0, 1, 0.3, 0.8]
)

VALID_OUTPUT_STRING = dict(
    mu1=['0.0', '1.0', '0.3', '0.8'],
    mu2=['0.0', '1.0', '0.3', '0.8'],
    kp=['0.0', '0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    kd=['0.0', '0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    maxContacts=['1', '2', '10', '10000'],
    minDepth=['0.0', '0.001', '0.1', '1.0'],
    maxVel=['0.0', '0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    selfCollide=['1', '0', '0', '1'],
    stopCfm=['0.0', '1.0', '0.3', '0.8'],
    stopErp=['0.0', '1.0', '0.3', '0.8']
)

DEFAULT_SCALAR_VALUES = dict(
    mu1=1,
    mu2=1,
    kp=1e12,
    kd=1,
    maxContacts=20,
    minDepth=0,
    maxVel=0.01,
    selfCollide=False,
    stopCfm=0,
    stopErp=0.2
)

INVALID_BASIC_VALUES = dict(
    mu1=[-1, dict(), True, None, list(), 'a', [1, 2]],
    mu2=[-1, dict(), True, None, list(), 'a', [1, 2]],
    kp=[-1, dict(), True, None, list(), 'a', [1, 2]],
    kd=[-1, dict(), True, None, list(), 'a', [1, 2]],
    maxContacts=[-1, dict(), True, None, list(), 'a', [1, 2]],
    minDepth=[-1, dict(), True, None, list(), 'a', [1, 2]],
    maxVel=[-1, dict(), True, None, list(), 'a', [1, 2]],
    selfCollide=[-1, 2, dict(), None, list(), 'a', [1, 2]],
    stopCfm=[-1, dict(), True, None, list(), 'a', [1, 2]],
    stopErp=[-1, dict(), True, None, list(), 'a', [1, 2]]
)

SDF_CONVERSION = dict(
    mu1=dict(ode=1, bullet=1),
    mu2=dict(ode=1, bullet=1),
    kp=dict(default=1e12),
    kd=dict(default=1),
    maxContacts=dict(default=20),
    minDepth=dict(default=0),
    maxVel=dict(default=0.01),
    selfCollide=dict(default=True),
    stopCfm=dict(default=0),
    stopErp=dict(default=0.2)
)

def get_urdf_file(name):
    return os.path.join(CUR_DIR, 'urdf', '{}.urdf'.format(name))
    

class TestURDFParser(unittest.TestCase):
    def test_basic_element_types(self):
        for urdf_tag in URDF_BASIC_OBJ_NAMES:
            obj = create_urdf_element(urdf_tag)

            self.assertIsNotNone(obj.xml_element_name, '{} is invalid'.format(urdf_tag))
            self.assertEqual(obj.xml_element_name, urdf_tag,
                             '{} has invalid URDF block name'.format(urdf_tag))
            self.assertTrue(obj.has_value(),
                            '{} should store a values'.format(urdf_tag))

            # Testing default value
            self.assertEqual(obj.value, DEFAULT_SCALAR_VALUES[urdf_tag],
                             '{} has wrong default value, value={}'.format(
                             urdf_tag, obj.value))

            for value, value_str in zip(VALID_BASIC_VALUES[urdf_tag], VALID_OUTPUT_STRING[urdf_tag]):
                obj.value = value
                self.assertEqual(
                    obj.value, value,
                    'New value for {} was not set, new_value={}'.format(
                        urdf_tag, value))

                self.assertEqual(obj.get_formatted_value_as_str(),
                    value_str,
                    'Wrong formatted output string for {}, expected={}, '
                    'returned={}'.format(
                        urdf_tag, value_str, obj.get_formatted_value_as_str()))

                # Test generation of XML block
                xml_element = obj.to_xml()
                self.assertEqual(xml_element.tag, urdf_tag,
                    'Invalid XML element tag for {}'.format(urdf_tag))
                self.assertEqual(xml_element.text, value_str,
                    'Invalid XML element tag for {}'.format(urdf_tag))

            for value in INVALID_BASIC_VALUES[urdf_tag]:
                with self.assertRaises(AssertionError):
                    obj.value = value

            for input_tag in SDF_CONVERSION[urdf_tag]:
                obj.value = SDF_CONVERSION[urdf_tag][input_tag]
                if input_tag == 'default':
                    sdf_obj = obj.to_sdf()
                else:
                    sdf_obj = obj.to_sdf(input_tag)
                self.assertIsNotNone(
                    sdf_obj,
                    '{} conversion to SDF returned an invalid object'.format(
                        urdf_tag))
                self.assertEqual(
                    sdf_obj.value,
                    SDF_CONVERSION[urdf_tag][input_tag],
                    'Converted SDF object for {} has the wrong set '
                    'value'.format(urdf_tag))

    # ====================================================
    # Testing geometry parsing
    # ====================================================
    def test_parse_geometry_box(self):
        filename = get_urdf_file('geometry_box')
        obj = parse_urdf(filename)

        self.assertTrue(obj.is_valid(), 'Box geometry should be valid')
        self.assertIsNotNone(obj.box, 'Box geometry should exist')
        self.assertEqual(obj.box.size, [1, 2, 3], 'Wrong box size, received={}, expected={}'.format(obj.box.size, [1, 2, 3]))

        sdf_obj = obj.to_sdf()
        self.assertIsNotNone(sdf_obj, 'Convert SDF box geometry is invalid')
        self.assertEqual(sdf_obj.box.size.value, obj.box.size, 'Converted SDF box has different value for size')

    def test_parse_geometry_sphere(self):
        filename = get_urdf_file('geometry_sphere')
        obj = parse_urdf(filename)

        self.assertTrue(obj.is_valid(), 'Sphere geometry should be valid')
        self.assertIsNotNone(obj.sphere, 'Sphere geometry should exist')
        self.assertEqual(obj.sphere.radius, 3.5, 'Wrong sphere radius, received={}, expected={}'.format(obj.sphere.radius, 3.5))

        sdf_obj = obj.to_sdf()
        self.assertIsNotNone(sdf_obj, 'Convert SDF sphere geometry is invalid')
        self.assertEqual(sdf_obj.sphere.radius.value, obj.sphere.radius, 'Converted SDF sphere has different value for radius')

    def test_parse_geometry_cylinder(self):
        filename = get_urdf_file('geometry_cylinder')
        obj = parse_urdf(filename)

        self.assertTrue(obj.is_valid(), 'Cylinder geometry should be valid')
        self.assertIsNotNone(obj.cylinder, 'Cylinder geometry should exist')
        self.assertEqual(obj.cylinder.radius, 3.4, 'Wrong cylinder radius, received={}, expected={}'.format(obj.cylinder.radius, 3.4))
        self.assertEqual(obj.cylinder.length, 5.6, 'Wrong cylinder radius, received={}, expected={}'.format(obj.cylinder.length, 5.6))

        sdf_obj = obj.to_sdf()
        self.assertIsNotNone(sdf_obj, 'Convert SDF cylinder geometry is invalid')
        self.assertEqual(sdf_obj.cylinder.radius.value, obj.cylinder.radius, 'Converted SDF cylinder has different value for radius')
        self.assertEqual(sdf_obj.cylinder.length.value, obj.cylinder.length, 'Converted SDF cylinder has different value for length')

    def test_parse_geometry_mesh(self):
        filename = get_urdf_file('geometry_mesh')
        obj = parse_urdf(filename)

        self.assertTrue(obj.is_valid(), 'Mesh geometry should be valid')
        self.assertIsNotNone(obj.mesh, 'Mesh geometry should exist')
        self.assertEqual(obj.mesh.filename, 'mesh', 'Wrong esh filename, received={}, expected={}'.format(obj.mesh.filename, 'mesh'))

        sdf_obj = obj.to_sdf()
        self.assertIsNotNone(sdf_obj, 'Convert SDF mesh geometry is invalid')
        self.assertEqual(sdf_obj.mesh.uri.value, obj.mesh.filename, 'Converted SDF mesh has different value for mesh')

    # ====================================================
    # Testing link parsing
    # ====================================================
    def test_parse_empty_link(self):
        filename = get_urdf_file('link_empty')
        obj = parse_urdf(filename)
        self.assertTrue(obj.is_valid(), 'Link should be valid')
        self.assertEqual(obj.name, 'empty_link', 'Link should have name empty link')
        self.assertIsNone(obj.mass, 'Link should have no mass')
        self.assertIsNone(obj.center_of_mass, 'Link should have no center of mass')
        self.assertIsNone(obj.inertia, 'Link should have no inertia')
        self.assertIsNone(obj.inertial, 'Link should have no inertial')
        self.assertIsNone(obj.collisions, 'Link should have no collisions')
        self.assertIsNone(obj.visuals, 'Link should have no visuals')    

    def test_parse_simple_link(self):
        filename = get_urdf_file('link_simple')
        obj = parse_urdf(filename)

        self.assertTrue(obj.is_valid(), 'Link should be valid')
      
        # Check that all elements are valid
        self.assertEqual(obj.name, 'simple_link', 'Link should have name simple link')
        self.assertIsNotNone(obj.mass, 'Link should have valid mass')
        self.assertIsNotNone(obj.center_of_mass, 'Link should have valid center of mass')
        self.assertIsNotNone(obj.inertia, 'Link should have valid inertia')
        self.assertIsNotNone(obj.inertial, 'Link should have valid inertial')
        self.assertIsNotNone(obj.collisions, 'Link should have one collision')
        self.assertIsNotNone(obj.visuals, 'Link should have one visual')

        # Check values
        self.assertEqual(obj.mass.value, 0.1, 'Wrong value for link mass, received={}, expected={}'.format(obj.mass.value, 0.1))
        self.assertEqual(obj.inertial.mass.value, 0.1, 'Wrong value for link mass in inertial, received={}, expected={}'.format(obj.inertial.mass.value, 0.1))
        self.assertEqual(obj.center_of_mass, [1, 2, 3], 'Wrong value for link.center_of_mass, received={}, expected={}'.format(obj.center_of_mass, [1, 2, 3]))
        self.assertEqual(obj.inertial.origin.xyz, [1, 2, 3], 'Wrong value for link.inertial.origin.xyz, received={}, expected={}'.format(obj.inertial.origin.xyz, [1, 2, 3]))
        self.assertEqual(obj.inertial.origin.rpy, [0, 0, 0], 'Wrong value for link.inertial.origin.rpy, received={}, expected={}'.format(obj.inertial.origin.rpy, [0, 0, 0]))
        
        # Check collision geometry
        self.assertEqual(len(obj.collisions), 1, 'Link should have one collision geometry')
        self.assertEqual(obj.collisions[0].origin.xyz, [4, 5, 6], 'Wrong collision origin.xyz, received={}, expected={}'.format(obj.collisions[0].origin.xyz, [4, 5, 6]))
        self.assertEqual(obj.collisions[0].origin.rpy, [1, 2, 3], 'Wrong collision origin.rpy, received={}, expected={}'.format(obj.collisions[0].origin.rpy, [1, 2, 3]))
        self.assertIsNotNone(obj.collisions[0].geometry.cylinder, 'Collision geometry should have a cylinder description')
        self.assertEqual(obj.collisions[0].geometry.cylinder.radius, 0.5, 'Wrong radius for collision cylinder, received={}, expected={}'.format(obj.collisions[0].geometry.cylinder.radius, 0.5))
        self.assertEqual(obj.collisions[0].geometry.cylinder.length, 0.5, 'Wrong radius for collision cylinder, received={}, expected={}'.format(obj.collisions[0].geometry.cylinder.length, 0.5))

        # Check visual geometry
        self.assertEqual(len(obj.visuals), 1, 'Link should have one visual geometry')
        self.assertEqual(obj.visuals[0].origin.xyz, [4, 5, 6], 'Wrong visual origin.xyz, received={}, expected={}'.format(obj.visuals[0].origin.xyz, [4, 5, 6]))
        self.assertEqual(obj.visuals[0].origin.rpy, [1, 2, 3], 'Wrong visual origin.rpy, received={}, expected={}'.format(obj.visuals[0].origin.rpy, [1, 2, 3]))
        self.assertIsNotNone(obj.visuals[0].geometry.mesh, 'Collision geometry should have a mesh description')
        self.assertEqual(obj.visuals[0].geometry.mesh.filename, 'mesh', 'Wrong visual mesh filename, received={}, expected={}'.format(obj.visuals[0].geometry.mesh.filename, 'mesh'))
        
    # ====================================================
    # Testing Gazebo block parsing
    # ====================================================

    def test_parse_gazebo_default_link(self):
        filename = get_urdf_file('gazebo_default_link')
        obj = parse_urdf(filename)

        self.assertTrue(obj.is_valid(), 'Gazebo block should be valid')
        self.assertEqual(obj.reference, 'link', 'Wrong value for reference, received={}, expected={}'.format(obj.reference, 'link'))
        self.assertEqual(obj.mu1.value, 1.0, 'Wrong value for mu1, received={}, expected={}'.format(obj.mu1.value, 1.0))
        self.assertEqual(obj.mu2.value, 1.0, 'Wrong value for mu2, received={}, expected={}'.format(obj.mu2.value, 1.0))
        self.assertEqual(obj.kp.value, 1000000.0, 'Wrong value for kp, received={}, expected={}'.format(obj.kp.value, 1000000.0))
        self.assertEqual(obj.kd.value, 100.0, 'Wrong value for kd, received={}, expected={}'.format(obj.kd.value, 100.0))
        self.assertEqual(obj.minDepth.value, 0.001, 'Wrong value for minDepth, received={}, expected={}'.format(obj.minDepth.value, 0.001))
        self.assertEqual(obj.maxVel.value, 1.0, 'Wrong value for maxVel, received={}, expected={}'.format(obj.maxVel.value, 1.0))
        self.assertEqual(obj.selfCollide.value, False, 'Wrong value for selfCollide, received={}, expected={}'.format(obj.selfCollide.value, False))
        self.assertEqual(obj.maxContacts.value, 15, 'Wrong value for maxContacts, received={}, expected={}'.format(obj.maxContacts.value, 15))

    def test_parse_gazebo_default_joint(self):
        filename = get_urdf_file('gazebo_default_joint')
        obj = parse_urdf(filename)

        self.assertTrue(obj.is_valid(), 'Gazebo block should be valid')
        self.assertEqual(obj.reference, 'joint', 'Wrong value for reference, received={}, expected={}'.format(obj.reference, 'joint'))
        self.assertEqual(obj.stopCfm.value, 0.1, 'Wrong value for stopCfm, received={}, expected={}'.format(obj.stopCfm.value, 0.1))
        self.assertEqual(obj.stopErp.value, 0.3, 'Wrong value for stopErp, received={}, expected={}'.format(obj.stopErp.value, 0.3))

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
            True, 'Wrong value for always on flag, received={}, expected={}'.format(
                sensor.always_on.value, True))
        self.assertIsNotNone(sensor.imu, 'IMU elements was not parsed')

        # Checking IMU parameters
        imu = obj.children['sensor'][0].imu
        self.assertIsNotNone(imu.noise, 'For SDF version 1.4, noise element should be valid')
        self.assertEqual(
            imu.noise.type.value, 
            'gaussian', 
            'Wrong value for type (SDF 1.4), received={}, expected={}'.format(
                imu.noise.type.value, True))
        self.assertEqual(
            imu.noise.accel.mean.value, 
            0.1, 
            'Wrong value for accel.mean value (SDF 1.4), received={}, expected={}'.format(
                imu.noise.accel.mean.value, 0.1))
        self.assertEqual(
            imu.noise.accel.bias_mean.value, 
            3, 
            'Wrong value for accel.bias_mean value (SDF 1.4), received={}, expected={}'.format(
                imu.noise.accel.bias_mean.value, 3))
        self.assertEqual(
            imu.noise.accel.stddev.value, 
            0.01, 
            'Wrong value for accel.stddev value (SDF 1.4), received={}, expected={}'.format(
                imu.noise.accel.stddev.value, 0.01))
        self.assertEqual(
            imu.noise.accel.bias_stddev.value, 
            3, 
            'Wrong value for accel.bias_stddev value (SDF 1.4), received={}, expected={}'.format(
                imu.noise.accel.bias_stddev.value, 3))
        self.assertEqual(
            imu.noise.rate.mean.value, 
            0.1, 
            'Wrong value for rate.mean value (SDF 1.4), received={}, expected={}'.format(
                imu.noise.rate.mean.value, 0.1))
        self.assertEqual(
            imu.noise.rate.bias_mean.value, 
            3, 
            'Wrong value for rate.bias_mean value (SDF 1.4), received={}, expected={}'.format(
                imu.noise.rate.bias_mean.value, 3))
        self.assertEqual(
            imu.noise.rate.stddev.value, 
            0.01, 
            'Wrong value for rate.stddev value (SDF 1.4), received={}, expected={}'.format(
                imu.noise.rate.stddev.value, 0.01))
        self.assertEqual(
            imu.noise.rate.bias_stddev.value, 
            3, 
            'Wrong value for rate.bias_stddev value (SDF 1.4), received={}, expected={}'.format(
                imu.noise.rate.bias_stddev.value, 3))

        # Checking plugin parameters
        self.assertIsNotNone(sensor.plugins, 'No plugin found for sensor')
        self.assertEqual(len(sensor.plugins), 1, 'There should be one plugin for sensor')

        plugin = sensor.plugins[0]
        print(plugin)
        self.assertEqual(
            plugin.name, 
            'some_imu_plugin', 
            'Wrong value for plugin.name value (SDF 1.4), received={}, expected={}'.format(
                plugin.name, 'some_imu_plugin'))
        self.assertEqual(
            plugin.filename, 
            'imu_ros_plugin.so', 
            'Wrong value for plugin.filename value (SDF 1.4), received={}, expected={}'.format(
                plugin.filename, 'imu_ros_plugin.so'))
        self.assertIn(
            'param', plugin.value,
            'Plugin parameter <param> not found')
        self.assertEqual(
            plugin.value['param'], 10,
            'Wrong value for plugin.param value (SDF 1.4), received={}, expected={}'.format(
                plugin.value['param'], 10))
        self.assertIn(
            'block', plugin.value,
            'Plugin parameter <block> not found')
        self.assertIn('subparam', plugin.value['block'],
            'Plugin block parameter <subparam> not found')
        self.assertEqual(
            plugin.value['block']['subparam'], 1.245,
            'Wrong value for plugin.block.subparam value (SDF 1.4), received={}, expected={}'.format(
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
            True, 'Wrong value for always on flag, received={}, expected={}'.format(
                sensor.always_on.value, True))
        self.assertIsNotNone(sensor.imu, 'IMU elements was not parsed')

        # Checking IMU parameters
        # Checking noise parameters
        imu = obj.children['sensor'][0].imu
        self.assertIsNotNone(imu.noise, 'For SDF version 1.5, noise element should be valid')
        self.assertEqual(
            imu.noise.type.value, 
            'gaussian', 
            'Wrong value for type (SDF 1.5), received={}, expected={}'.format(
                imu.noise.type.value, True))
        self.assertEqual(
            imu.noise.accel.mean.value, 
            0.2, 
            'Wrong value for accel.mean value (SDF 1.5), received={}, expected={}'.format(
                imu.noise.accel.mean.value, 0.2))
        self.assertEqual(
            imu.noise.accel.bias_mean.value, 
            3, 
            'Wrong value for accel.bias_mean value (SDF 1.5), received={}, expected={}'.format(
                imu.noise.accel.bias_mean.value, 3))
        self.assertEqual(
            imu.noise.accel.stddev.value, 
            0.02, 
            'Wrong value for accel.stddev value (SDF 1.5), received={}, expected={}'.format(
                imu.noise.accel.stddev.value, 0.02))
        self.assertEqual(
            imu.noise.accel.bias_stddev.value, 
            3, 
            'Wrong value for accel.bias_stddev value (SDF 1.5), received={}, expected={}'.format(
                imu.noise.accel.bias_stddev.value, 3))
        self.assertEqual(
            imu.noise.rate.mean.value, 
            0.2, 
            'Wrong value for rate.mean value (SDF 1.5), received={}, expected={}'.format(
                imu.noise.rate.mean.value, 0.2))
        self.assertEqual(
            imu.noise.rate.bias_mean.value, 
            3, 
            'Wrong value for rate.bias_mean value (SDF 1.5), received={}, expected={}'.format(
                imu.noise.rate.bias_mean.value, 3))
        self.assertEqual(
            imu.noise.rate.stddev.value, 
            0.02, 
            'Wrong value for rate.stddev value (SDF 1.5), received={}, expected={}'.format(
                imu.noise.rate.stddev.value, 0.02))
        self.assertEqual(
            imu.noise.rate.bias_stddev.value, 
            3, 
            'Wrong value for rate.bias_stddev value (SDF 1.5), received={}, expected={}'.format(
                imu.noise.rate.bias_stddev.value, 3))

        # Checking noise parameters for angular velocity and linear acceleration
        for imu_element in ['angular_velocity', 'linear_acceleration']:
            self.assertTrue(
                    hasattr(imu, imu_element), 
                    'IMU has no element {}'.format(imu_element))
            for component in ['x', 'y', 'z']:                
                self.assertTrue(
                    hasattr(getattr(imu, imu_element), component), 
                    'imu.{} has no element {}'.format(imu_element, component))

                self.assertTrue(
                    hasattr(getattr(getattr(imu, imu_element), component), 'noise'),
                    'imu.{}.{} has no element noise'.format(imu_element, component))

                noise = getattr(getattr(imu, imu_element), component).noise

                if component == 'x':
                    self.assertEqual(
                        noise.type.value, 
                        'none',
                        'Wrong value for noise imu.{}.{}.noise.type, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.type.value,
                            'none'))
                elif component == 'y':
                    self.assertEqual(
                        noise.type.value, 
                        'gaussian',
                        'Wrong value for noise imu.{}.{}.noise.type, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.type.value,
                            'gaussian'))
                elif component == 'z':
                    self.assertEqual(
                        noise.type.value, 
                        'gaussian_quantized',
                        'Wrong value for noise imu.{}.{}.noise.type, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.type.value,
                            'gaussian_quantized'))

                self.assertEqual(
                    noise.mean.value,
                    0.2,
                    'Wrong value for noise imu.{}.{}.noise.mean, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.mean.value,
                            0.2))
                self.assertEqual(
                    noise.stddev.value,
                    0.02,
                    'Wrong value for noise imu.{}.{}.noise.stddev, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.stddev.value,
                            0.02))
                self.assertEqual(
                    noise.bias_mean.value,
                    4,
                    'Wrong value for noise imu.{}.{}.noise.bias_mean, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.bias_mean.value,
                            4))
                self.assertEqual(
                    noise.bias_stddev.value,
                    4,
                    'Wrong value for noise imu.{}.{}.noise.bias_stddev, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.bias_stddev.value,
                            4))
                self.assertEqual(
                    noise.precision.value,
                    0.5,
                    'Wrong value for noise imu.{}.{}.noise.precision, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.precision.value,
                            0.5))

        # Checking plugin parameters
        self.assertIsNotNone(sensor.plugins, 'No plugin found for sensor')
        self.assertEqual(len(sensor.plugins), 1, 'There should be one plugin for sensor')

        plugin = sensor.plugins[0]
        self.assertEqual(
            plugin.name, 
            'some_imu_plugin', 
            'Wrong value for plugin.name value (SDF 1.5), received={}, expected={}'.format(
                plugin.name, 'some_imu_plugin'))
        self.assertEqual(
            plugin.filename, 
            'imu_ros_plugin.so', 
            'Wrong value for plugin.filename value (SDF 1.5), received={}, expected={}'.format(
                plugin.filename, 'imu_ros_plugin.so'))
        self.assertIn(
            'param', plugin.value,
            'Plugin parameter <param> not found')
        self.assertEqual(
            plugin.value['param'], 10,
            'Wrong value for plugin.param value (SDF 1.5), received={}, expected={}'.format(
                plugin.value['param'], 10))
        self.assertIn(
            'block', plugin.value,
            'Plugin parameter <block> not found')
        self.assertIn('subparam', plugin.value['block'],
            'Plugin block parameter <subparam> not found')
        self.assertEqual(
            plugin.value['block']['subparam'], 1.245,
            'Wrong value for plugin.block.subparam value (SDF 1.5), received={}, expected={}'.format(
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
            True, 'Wrong value for always on flag, received={}, expected={}'.format(
                sensor.always_on.value, True))
        self.assertIsNotNone(sensor.imu, 'IMU elements was not parsed')

        # Checking IMU parameters        
        imu = obj.children['sensor'][0].imu
        self.assertEqual(
            imu.topic.value, 'imu_topic',
            'Wrong value for imu.topic, received={}, expected={}'.format(
                imu.topic.value, 'imu_topic'))

        self.assertIsNotNone(imu.orientation_reference_frame.localization, 
            'No localization element found for orientation_reference_frame')
        # Checking the IMU's orientation reference frame description
        self.assertEqual(
            imu.orientation_reference_frame.localization.value,
            'CUSTOM',
            'Wrong value for imu.orientation_reference_frame.localization,'
            ' received={}, expected={}'.format(
                imu.orientation_reference_frame.localization.value, 'CUSTOM'))
        
        # Checking noise parameters for angular velocity and linear acceleration
        for imu_element in ['angular_velocity', 'linear_acceleration']:
            self.assertTrue(
                    hasattr(imu, imu_element), 
                    'IMU has no element {}'.format(imu_element))
            for component in ['x', 'y', 'z']:                
                self.assertTrue(
                    hasattr(getattr(imu, imu_element), component), 
                    'imu.{} has no element {}'.format(imu_element, component))

                self.assertTrue(
                    hasattr(getattr(getattr(imu, imu_element), component), 'noise'),
                    'imu.{}.{} has no element noise'.format(imu_element, component))

                noise = getattr(getattr(imu, imu_element), component).noise

                if component == 'x':
                    self.assertEqual(
                        noise.type.value, 
                        'none',
                        'Wrong value for noise imu.{}.{}.noise.type, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.type.value,
                            'none'))
                elif component == 'y':
                    self.assertEqual(
                        noise.type.value, 
                        'gaussian',
                        'Wrong value for noise imu.{}.{}.noise.type, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.type.value,
                            'gaussian'))
                elif component == 'z':
                    self.assertEqual(
                        noise.type.value, 
                        'gaussian_quantized',
                        'Wrong value for noise imu.{}.{}.noise.type, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.type.value,
                            'gaussian_quantized'))

                self.assertEqual(
                    noise.mean.value,
                    0.2,
                    'Wrong value for noise imu.{}.{}.noise.mean, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.mean.value,
                            0.2))
                self.assertEqual(
                    noise.stddev.value,
                    0.02,
                    'Wrong value for noise imu.{}.{}.noise.stddev, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.stddev.value,
                            0.02))
                self.assertEqual(
                    noise.bias_mean.value,
                    4,
                    'Wrong value for noise imu.{}.{}.noise.bias_mean, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.bias_mean.value,
                            4))
                self.assertEqual(
                    noise.bias_stddev.value,
                    4,
                    'Wrong value for noise imu.{}.{}.noise.bias_stddev, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.bias_stddev.value,
                            4))
                self.assertEqual(
                    noise.precision.value,
                    0.5,
                    'Wrong value for noise imu.{}.{}.noise.precision, received={}, expected={}'.format(
                            imu_element,
                            component,
                            noise.precision.value,
                            0.5))

        # Checking plugin parameters
        self.assertIsNotNone(sensor.plugins, 'No plugin found for sensor')
        self.assertEqual(len(sensor.plugins), 1, 'There should be one plugin for sensor')

        plugin = sensor.plugins[0]
        self.assertEqual(
            plugin.name, 
            'some_imu_plugin', 
            'Wrong value for plugin.name value (SDF 1.6), received={}, expected={}'.format(
                plugin.name, 'some_imu_plugin'))
        self.assertEqual(
            plugin.filename, 
            'imu_ros_plugin.so', 
            'Wrong value for plugin.filename value (SDF 1.6), received={}, expected={}'.format(
                plugin.filename, 'imu_ros_plugin.so'))
        self.assertIn(
            'param', plugin.value,
            'Plugin parameter <param> not found')
        self.assertEqual(
            plugin.value['param'], 10,
            'Wrong value for plugin.param value (SDF 1.6), received={}, expected={}'.format(
                plugin.value['param'], 10))
        self.assertIn(
            'block', plugin.value,
            'Plugin parameter <block> not found')
        self.assertIn('subparam', plugin.value['block'],
            'Plugin block parameter <subparam> not found')
        self.assertEqual(
            plugin.value['block']['subparam'], 1.245,
            'Wrong value for plugin.block.subparam value (SDF 1.6), received={}, expected={}'.format(
                plugin.value['block']['subparam'], 1.245))

                

if __name__ == '__main__':
    unittest.main()
