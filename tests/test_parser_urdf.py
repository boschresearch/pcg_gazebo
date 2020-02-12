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
from pcg_gazebo.parsers import parse_urdf


CUR_DIR = os.path.dirname(os.path.abspath(__file__))


def get_urdf_file(name):
    return os.path.join(CUR_DIR, 'urdf', '{}.urdf'.format(name))


class TestURDFParser(unittest.TestCase):
    def test_parse_geometry_box(self):
        filename = get_urdf_file('geometry_box')
        obj = parse_urdf(filename)

        self.assertTrue(obj.is_valid(), 'Box geometry should be valid')
        self.assertIsNotNone(obj.box, 'Box geometry should exist')
        self.assertEqual(
            obj.box.size, [
                1, 2, 3], 'Wrong box size, received={}, expected={}'.format(
                obj.box.size, [
                    1, 2, 3]))

        sdf_obj = obj.to_sdf()
        self.assertIsNotNone(sdf_obj, 'Convert SDF box geometry is invalid')
        self.assertEqual(
            sdf_obj.box.size.value,
            obj.box.size,
            'Converted SDF box has different value for size')

    def test_parse_geometry_sphere(self):
        filename = get_urdf_file('geometry_sphere')
        obj = parse_urdf(filename)

        self.assertTrue(obj.is_valid(), 'Sphere geometry should be valid')
        self.assertIsNotNone(obj.sphere, 'Sphere geometry should exist')
        self.assertEqual(
            obj.sphere.radius,
            3.5,
            'Wrong sphere radius, received={}, expected={}'.format(
                obj.sphere.radius,
                3.5))

        sdf_obj = obj.to_sdf()
        self.assertIsNotNone(sdf_obj, 'Convert SDF sphere geometry is invalid')
        self.assertEqual(
            sdf_obj.sphere.radius.value,
            obj.sphere.radius,
            'Converted SDF sphere has different value for radius')

    def test_parse_geometry_cylinder(self):
        filename = get_urdf_file('geometry_cylinder')
        obj = parse_urdf(filename)

        self.assertTrue(obj.is_valid(), 'Cylinder geometry should be valid')
        self.assertIsNotNone(obj.cylinder, 'Cylinder geometry should exist')
        self.assertEqual(
            obj.cylinder.radius,
            3.4,
            'Wrong cylinder radius, received={}, expected={}'.format(
                obj.cylinder.radius,
                3.4))
        self.assertEqual(
            obj.cylinder.length,
            5.6,
            'Wrong cylinder radius, received={}, expected={}'.format(
                obj.cylinder.length,
                5.6))

        sdf_obj = obj.to_sdf()
        self.assertIsNotNone(
            sdf_obj, 'Convert SDF cylinder geometry is invalid')
        self.assertEqual(
            sdf_obj.cylinder.radius.value,
            obj.cylinder.radius,
            'Converted SDF cylinder has different value for radius')
        self.assertEqual(
            sdf_obj.cylinder.length.value,
            obj.cylinder.length,
            'Converted SDF cylinder has different value for length')

    def test_parse_geometry_mesh(self):
        filename = get_urdf_file('geometry_mesh')
        obj = parse_urdf(filename)

        self.assertTrue(obj.is_valid(), 'Mesh geometry should be valid')
        self.assertIsNotNone(obj.mesh, 'Mesh geometry should exist')
        self.assertEqual(
            obj.mesh.filename,
            'mesh',
            'Wrong esh filename, received={}, expected={}'.format(
                obj.mesh.filename,
                'mesh'))

        sdf_obj = obj.to_sdf()
        self.assertIsNotNone(sdf_obj, 'Convert SDF mesh geometry is invalid')
        self.assertEqual(
            sdf_obj.mesh.uri.value,
            obj.mesh.filename,
            'Converted SDF mesh has different value for mesh')

    # ====================================================
    # Testing link parsing
    # ====================================================
    def test_parse_empty_link(self):
        filename = get_urdf_file('link_empty')
        obj = parse_urdf(filename)
        self.assertTrue(obj.is_valid(), 'Link should be valid')
        self.assertEqual(
            obj.name,
            'empty_link',
            'Link should have name empty link')
        self.assertIsNone(obj.mass, 'Link should have no mass')
        self.assertIsNone(
            obj.center_of_mass,
            'Link should have no center of mass')
        self.assertIsNone(obj.inertia, 'Link should have no inertia')
        self.assertIsNone(obj.inertial, 'Link should have no inertial')
        self.assertIsNone(obj.collisions, 'Link should have no collisions')
        self.assertIsNone(obj.visuals, 'Link should have no visuals')

    def test_parse_simple_link(self):
        filename = get_urdf_file('link_simple')
        obj = parse_urdf(filename)

        self.assertTrue(obj.is_valid(), 'Link should be valid')

        # Check that all elements are valid
        self.assertEqual(
            obj.name,
            'simple_link',
            'Link should have name simple link')
        self.assertIsNotNone(obj.mass, 'Link should have valid mass')
        self.assertIsNotNone(
            obj.center_of_mass,
            'Link should have valid center of mass')
        self.assertIsNotNone(obj.inertia, 'Link should have valid inertia')
        self.assertIsNotNone(obj.inertial, 'Link should have valid inertial')
        self.assertIsNotNone(obj.collisions, 'Link should have one collision')
        self.assertIsNotNone(obj.visuals, 'Link should have one visual')

        # Check values
        self.assertEqual(
            obj.mass.value, 0.1,
            'Wrong value for link mass, received={}, '
            'expected={}'.format(obj.mass.value, 0.1))
        self.assertEqual(
            obj.inertial.mass.value, 0.1,
            'Wrong value for link mass in inertial, '
            'received={}, expected={}'.format(
                obj.inertial.mass.value, 0.1))
        self.assertEqual(
            obj.center_of_mass, [1, 2, 3],
            'Wrong value for link.center_of_mass, '
            'received={}, expected={}'.format(
                obj.center_of_mass, [1, 2, 3]))
        self.assertEqual(
            obj.inertial.origin.xyz, [1, 2, 3],
            'Wrong value for link.inertial.origin.xyz, '
            'received={}, expected={}'.format(
                obj.inertial.origin.xyz, [1, 2, 3]))
        self.assertEqual(
            obj.inertial.origin.rpy, [0, 0, 0],
            'Wrong value for link.inertial.origin.rpy, '
            'received={}, expected={}'.format(
                obj.inertial.origin.rpy, [0, 0, 0]))

        # Check collision geometry
        self.assertEqual(len(obj.collisions), 1,
                         'Link should have one collision geometry')
        self.assertEqual(
            obj.collisions[0].origin.xyz, [4, 5, 6],
            'Wrong collision origin.xyz, received={}, expected={}'.format(
                obj.collisions[0].origin.xyz, [4, 5, 6]))
        self.assertEqual(
            obj.collisions[0].origin.rpy, [1, 2, 3],
            'Wrong collision origin.rpy, received={}, expected={}'.format(
                obj.collisions[0].origin.rpy, [1, 2, 3]))
        self.assertIsNotNone(
            obj.collisions[0].geometry.cylinder,
            'Collision geometry should have a cylinder description')
        self.assertEqual(
            obj.collisions[0].geometry.cylinder.radius,
            0.5,
            'Wrong radius for collision cylinder, '
            'received={}, expected={}'.format(
                obj.collisions[0].geometry.cylinder.radius,
                0.5))
        self.assertEqual(
            obj.collisions[0].geometry.cylinder.length,
            0.5,
            'Wrong radius for collision cylinder, '
            'received={}, expected={}'.format(
                obj.collisions[0].geometry.cylinder.length,
                0.5))

        # Check visual geometry
        self.assertEqual(len(obj.visuals), 1,
                         'Link should have one visual geometry')
        self.assertEqual(
            obj.visuals[0].origin.xyz, [4, 5, 6],
            'Wrong visual origin.xyz, received={}, expected={}'.format(
                obj.visuals[0].origin.xyz, [4, 5, 6]))
        self.assertEqual(
            obj.visuals[0].origin.rpy, [1, 2, 3],
            'Wrong visual origin.rpy, '
            'received={}, expected={}'.format(
                obj.visuals[0].origin.rpy, [1, 2, 3]))
        self.assertIsNotNone(
            obj.visuals[0].geometry.mesh,
            'Collision geometry should have a mesh description')
        self.assertEqual(
            obj.visuals[0].geometry.mesh.filename,
            'mesh',
            'Wrong visual mesh filename, received={}, expected={}'.format(
                obj.visuals[0].geometry.mesh.filename,
                'mesh'))


if __name__ == '__main__':
    unittest.main()
