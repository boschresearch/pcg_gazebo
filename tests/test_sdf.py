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
import sys
import unittest
from pcg_gazebo.parsers.sdf import create_sdf_element, \
    get_all_sdf_element_classes
from pcg_gazebo.parsers.sdf_config import \
    create_sdf_config_element, get_all_sdf_config_element_classes
from pcg_gazebo.parsers.urdf import create_urdf_element

INVALID_INERTIA_VALUES = [
    None,
    'a',
    '2',
    -1,
    0,
    [1],
    [1, 1, 1],
    dict(ixx=1, ixy='a', ixz=1, iyy=3, iyz=1, izz=1),
    dict(ixx=None, ixy=1, ixz=1, iyy=3, iyz=1, a=1)
]

VALID_INERTIA_VALUES = [
    dict(ixx=1, ixy=2, ixz=3, iyy=4, iyz=5, izz=6),
    dict(ixx=0, ixy=0, ixz=0, iyy=0, iyz=0, izz=0)
]

INVALID_SIZE_VALUES = [None, dict(), [-1, 0, 0], ['a', 0, 0], [0, 0]]

VALID_SIZE_VALUES = [[0 for _ in range(3)], [1, 2, 3], [1, 1, 1]]

INVALID_MASSES = [None, 'a', '2', -10]

INVALID_POSE_VALUES = [
    None,
    dict,
    [-1, 0, 0],
    ['a', 0, 0],
    [0, 0],
    [None for _ in range(6)],
    ['a' for _ in range(6)]
]

VALID_BOX_PROPS = dict(
    size=[1, 1, 1]
)

VALID_IMAGE_PROPS = dict(
    uri='test_uri',
    scale=[1],
    threshold=100,
    granularity=10,
    height=1.2
)

VALID_CYLINDER_PROPS = dict(
    radius=1,
    length=1
)

VALID_SPHERE_PROPS = dict(
    radius=1
)

VALID_MESH_PROPS = dict(
    uri='test_uri',
    scale=[2, 2, 2],
    submesh=dict(name='test_submesh', center=True)
)

VALID_PLANE_PROPS = dict(
    size=[2, 3],
    normal=[1, 0, 0]
)

VALID_MATERIAL_PROPS = dict(
    script=dict(name='test', uri='test_uri'),
    shader=dict(attributes=dict(type='vertex'), normal_map='test_normal_map'),
    lighting=True,
    ambient=[0, 0, 0, 1],
    diffuse=[0, 0, 0, 1],
    specular=[0, 0, 0, 1],
    emissive=[0, 0, 0, 1]
)

VALID_VISUAL_PROPS = dict(
    name='test',
    pose=[1 for _ in range(6)],
    transparency=0.5,
    cast_shadows=False
)

VALID_ODE_FRICTION_MODEL_OPTIONS = [
    'pyramid_model',
    'box_model',
    'cone_model'
]

DEFAULT_ODE_SOLVER = dict(
    min_step_size=0.0001,
    iters=50,
    sor=1.3,
    type='quick',
    precon_iters=0,
    use_dynamic_moi_rescaling=False,
    friction_model='pyramid_model'
)

DEFAULT_BULLET_SOLVER = dict(
    type='sequential_impulse',
    min_step_size=0.0001,
    iters=50,
    sor=1.3
)

SDF_BASIC_OBJ_NAMES = [
    'mu',
    'mu2',
    'accuracy',
    'self_collide',
    'allow_auto_disable',
    'always_on',
    'bias_mean',
    'bias_stddev',
    'cast_shadows',
    'center',
    'cfm_damping',
    'cfm',
    'child',
    'coefficient',
    'contact_max_correcting_vel',
    'contact_surface_layer',
    'damping',
    'diffuse',
    'dissipation',
    'dynamic_friction',
    'effort',
    'emissive',
    'enable_wind',
    'erp',
    'far',
    'fdir1',
    'format',
    'friction_model',
    'friction2',
    'granularity',
    'height',
    'horizontal_fov',
    'initial_position',
    'iters',
    'ixx',
    'ixy',
    'ixz',
    'iyy',
    'iyz',
    'izz',
    'uri'
]

VALID_BASIC_VALUES = dict(
    mu=[0, 1, 0.3, 0.8, 1.0],
    mu2=[0, 1, 0.3, 0.8, 1.0],
    accuracy=[0.1, 0.3, 0.8, 1, 10, 10000],
    self_collide=[True, False, 0, 1],
    allow_auto_disable=[True, False, 0, 1],
    always_on=[True, False, 0, 1],
    ambient=[[0 for _ in range(4)], [1 for _ in range(4)],
             [0.5 for _ in range(4)]],
    bias_mean=[0, 0.5, 1, 2.5, 10],
    bias_stddev=[0, 0.5, 1, 2.5, 10],
    cast_shadows=[True, False, 0, 1],
    center=[True, False, 0, 1],
    cfm_damping=[True, False, 0, 1],
    cfm=[0, 0.5, 1, 2.5, 10],
    child=['none', 'test', 'a', ''],
    coefficient=[0, 1, 0.3, 0.8, 1.0],
    contact_max_correcting_vel=[0, 0.1, 0.3, 0.8, 1, 10, 10000],
    contact_surface_layer=[0, 0.1, 0.3, 0.8, 1, 10, 10000],
    damping=[0, 0.1, 0.3, 0.8, 1, 10, 10000],
    diffuse=[[0 for _ in range(4)], [1 for _ in range(4)],
             [0.5 for _ in range(4)]],
    dissipation=[0.1, 0.3, 0.8, 1, 10, 10000],
    dynamic_friction=[0, 0.1, 0.3, 0.8, 1, 10, 10000],
    effort=[-1, 0, 0.1, 0.3, 0.8, 1, 10, 10000],
    emissive=[[0 for _ in range(4)], [1 for _ in range(4)],
              [0.5 for _ in range(4)]],
    empty=[],
    enable_wind=[True, False, 0, 1],
    erp=[0, 0.5, 1, 2.5, 10],
    far=[0, 0.1, 0.3, 0.8, 1, 10, 10000],
    fdir1=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    format=['L8', 'R8G8B8', 'B8G8R8', 'BAYER_RGGB8', 'BAYER_BGGR8',
            'BAYER_GBRG8', 'BAYER_GRBG8'],
    friction_model=['pyramid_model', 'box_model', 'cone_model'],
    friction2=[0, 1, 0.3, 0.8, 1.0],
    granularity=[0, 1, 2],
    height=[0.1, 0.3, 0.8, 1, 10, 10000],
    horizontal_fov=[0.0, 0.1, 0.3, 0.8, 1, 10, 10000],
    initial_position=[-2.0, -1.0, -0.5, -0.1, 0.0, 0.1, 0.5, 1.0, 2.0],
    iters=[1, 2, 50, 1000],
    ixx=[0, 0.5, 1, 2.5, 10],
    ixy=[-2, -1.5, -0.5, 0, 0.5, 1.5, 2.0],
    ixz=[-2, -1.5, -0.5, 0, 0.5, 1.5, 2.0],
    iyy=[0, 0.5, 1, 2.5, 10],
    iyz=[-2, -1.5, -0.5, 0, 0.5, 1.5, 2.0],
    izz=[0, 0.5, 1, 2.5, 10],
    uri=['__default__', 'asdf']
)

VALID_OUTPUT_STRING = dict(
    mu=['0.0', '1.0', '0.3', '0.8', '1.0'],
    mu2=['0.0', '1.0', '0.3', '0.8', '1.0'],
    accuracy=['0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    self_collide=['1', '0', '0', '1'],
    allow_auto_disable=['1', '0', '0', '1'],
    always_on=['1', '0', '0', '1'],
    ambient=['0 0 0 0', '1 1 1 1', '0.5 0.5 0.5 0.5'],
    bias_mean=['0.0', '0.5', '1.0', '2.5', '10.0'],
    bias_stddev=['0.0', '0.5', '1.0', '2.5', '10.0'],
    cast_shadows=['1', '0', '0', '1'],
    center=['1', '0', '0', '1'],
    cfm_damping=['1', '0', '0', '1'],
    cfm=['0.0', '0.5', '1.0', '2.5', '10.0'],
    child=['none', 'test', 'a', ''],
    coefficient=['0.0', '1.0', '0.3', '0.8', '1.0'],
    contact_max_correcting_vel=['0.0', '0.1', '0.3', '0.8', '1.0',
                                '10.0', '10000.0'],
    contact_surface_layer=['0.0', '0.1', '0.3', '0.8', '1.0',
                           '10.0', '10000.0'],
    damping=['0.0', '0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    diffuse=['0 0 0 0', '1 1 1 1', '0.5 0.5 0.5 0.5'],
    dissipation=['0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    dynamic_friction=['0.0', '0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    effort=['-1.0', '0.0', '0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    emissive=['0 0 0 0', '1 1 1 1', '0.5 0.5 0.5 0.5'],
    empty=[],
    enable_wind=['1', '0', '0', '1'],
    erp=['0.0', '0.5', '1.0', '2.5', '10.0'],
    far=['0.0', '0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    fdir1=['1 0 0', '0 1 0', '0 0 1'],
    format=['L8', 'R8G8B8', 'B8G8R8', 'BAYER_RGGB8', 'BAYER_BGGR8',
            'BAYER_GBRG8', 'BAYER_GRBG8'],
    friction_model=['pyramid_model', 'box_model', 'cone_model'],
    friction2=['0.0', '1.0', '0.3', '0.8', '1.0'],
    granularity=['0', '1', '2'],
    height=['0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    horizontal_fov=['0.0', '0.1', '0.3', '0.8', '1.0', '10.0', '10000.0'],
    initial_position=['-2.0', '-1.0', '-0.5', '-0.1', '0.0',
                      '0.1', '0.5', '1.0', '2.0'],
    iters=['1', '2', '50', '1000'],
    ixx=['0.0', '0.5', '1.0', '2.5', '10.0'],
    ixy=['-2.0', '-1.5', '-0.5', '0.0', '0.5', '1.5', '2.0'],
    ixz=['-2.0', '-1.5', '-0.5', '0.0', '0.5', '1.5', '2.0'],
    iyy=['0.0', '0.5', '1.0', '2.5', '10.0'],
    iyz=['-2.0', '-1.5', '-0.5', '0.0', '0.5', '1.5', '2.0'],
    izz=['0.0', '0.5', '1.0', '2.5', '10.0'],
    uri=['__default__', 'asdf']
)

INVALID_BASIC_VALUES = dict(
    mu=[-1, dict(), True, None, list(), 'a', [1, 2]],
    mu2=[-1, dict(), True, None, list(), 'a', [1, 2]],
    accuracy=[-1, dict(), True, None, list(), 'a', [1, 2]],
    self_collide=[-1, 2, dict(), None, list(), 'a', [1, 2]],
    allow_auto_disable=[-1, 2, dict(), None, list(), 'a', [1, 2]],
    always_on=[-1, 2, dict(), None, list(), 'a', [1, 2]],
    ambient=[-1, 2, dict(), None, True, list(), 'a',
             [1, 2], [True, False, None, 'a']],
    bias_mean=[-1, dict(), True, None, list(), 'a', [1, 2]],
    bias_stddev=[-1, dict(), True, None, list(), 'a', [1, 2]],
    cast_shadows=[-1, 2, dict(), None, list(), 'a', [1, 2]],
    center=[-1, 2, dict(), None, list(), 'a', [1, 2, 3]],
    cfm_damping=[-1, 2, dict(), None, list(), 'a', [1, 2, 3]],
    cfm=[-1, dict(), True, None, list(), 'a', [1, 2]],
    child=[-1, 2, dict(), None, True, list(), [1, 2],
           [True, False, None, 'a']],
    coefficient=[-1, 2, dict(), True, None, list(), 'a', [1, 2]],
    contact_max_correcting_vel=[-1, dict(), True, None, list(), 'a', [1, 2]],
    contact_surface_layer=[-1, dict(), True, None, list(), 'a', [1, 2]],
    damping=[-1, dict(), True, None, list(), 'a', [1, 2]],
    diffuse=[-1, 2, dict(), None, True, list(),
             [1, 2], [True, False, None, 'a']],
    dissipation=[-1, dict(), True, None, list(), 'a', [1, 2]],
    dynamic_friction=[-1, dict(), True, None, list(), 'a', [1, 2]],
    effort=[dict(), True, None, list(), 'a', [1, 2]],
    emissive=[-1, 2, dict(), None, True, list(), 'a',
              [1, 2], [True, False, None, 'a']],
    enable_wind=[-1, 2, dict(), None, list(), 'a', [1, 2]],
    erp=[-1, dict(), True, None, list(), 'a', [1, 2]],
    far=[-1, dict(), True, None, list(), 'a', [1, 2]],
    fdir1=[-1, 2, dict(), None, True, list(), 'a',
           [1, 2], [True, False, None, 'a']],
    format=[-1, 2, dict(), None, True, list(), [1, 2],
            [True, False, None, 'a']],
    friction_model=[-1, 2, dict(), None, True, list(), [1, 2],
                    [True, False, None, 'a']],
    friction2=[-1, dict(), True, None, list(), 'a', [1, 2]],
    granularity=[-1, dict(), True, None, list(), 'a', [1, 2]],
    height=[-1, dict(), True, None, list(), 'a', [1, 2]],
    horizontal_fov=[-1, dict(), True, None, list(), 'a', [1, 2]],
    initial_position=[dict(), True, None, list(), 'a', [1, 2]],
    iters=[-1, 0, 2.5, dict(), True, None, list(), 'a', [1, 2]],
    ixx=[dict(), True, None, list(), 'a', [1, 2]],
    ixy=[dict(), True, None, list(), 'a', [1, 2]],
    ixz=[dict(), True, None, list(), 'a', [1, 2]],
    iyy=[dict(), True, None, list(), 'a', [1, 2]],
    iyz=[dict(), True, None, list(), 'a', [1, 2]],
    izz=[dict(), True, None, list(), 'a', [1, 2]],
    uri=[dict(), True, None, list(), [1, 2]]
)

DEFAULT_BASIC_VALUES = dict(
    mu=1,
    mu2=1,
    accuracy=0.001,
    self_collide=False,
    allow_auto_disable=False,
    always_on=False,
    ambient=[0, 0, 0, 1],
    bias_mean=0,
    bias_stddev=0,
    cast_shadows=True,
    center=False,
    cfm_damping=False,
    cfm=0,
    erp=0.2,
    child='none',
    coefficient=1,
    contact_max_correcting_vel=100,
    contact_surface_layer=0.001,
    damping=0,
    diffuse=[0, 0, 0, 1],
    dissipation=100,
    dynamic_friction=0.9,
    effort=-1,
    emissive=[0, 0, 0, 1],
    enable_wind=False,
    far=0,
    fdir1=[0, 0, 0],
    format='R8G8B8',
    friction_model='pyramid_model',
    friction2=1,
    granularity=1,
    height=1,
    horizontal_fov=1.047,
    initial_position=0,
    iters=50,
    ixx=0,
    ixy=0,
    ixz=0,
    iyy=0,
    iyz=0,
    izz=0,
    uri='__default__'
)


class TestSDFParser(unittest.TestCase):
    def test_basic_element_types(self):
        for sdf_tag in SDF_BASIC_OBJ_NAMES:
            obj = create_sdf_element(sdf_tag)

            self.assertIsNotNone(
                obj.xml_element_name,
                '{} is invalid'.format(sdf_tag))
            self.assertEqual(obj.xml_element_name, sdf_tag,
                             '{} has invalid sdf block name'.format(sdf_tag))
            self.assertTrue(obj.has_value(),
                            '{} should store a values'.format(sdf_tag))

            # Testing default value
            self.assertEqual(obj.value, DEFAULT_BASIC_VALUES[sdf_tag],
                             '{} has wrong default value, value={}'.format(
                             sdf_tag, obj.value))

            for value, value_str in zip(
                    VALID_BASIC_VALUES[sdf_tag], VALID_OUTPUT_STRING[sdf_tag]):
                obj.value = value
                self.assertEqual(
                    obj.value, value,
                    'New value for {} was not set, new_value={}'.format(
                        sdf_tag, value))

                self.assertEqual(
                    obj.get_formatted_value_as_str(),
                    value_str,
                    'Wrong formatted output string for {}, '
                    'expected={}, returned={}'.format(
                        sdf_tag,
                        value_str,
                        obj.get_formatted_value_as_str()))

                # Test generation of XML block
                xml_element = obj.to_xml()
                self.assertEqual(
                    xml_element.tag,
                    sdf_tag,
                    'Invalid XML element tag for {}'.format(sdf_tag))
                self.assertEqual(
                    xml_element.text,
                    value_str,
                    'Invalid XML element tag for {}'.format(sdf_tag))

                xml_str = obj.to_xml_as_str()
                output_str = '<{}>{}</{}>'.format(sdf_tag, value_str, sdf_tag)
                self.assertEqual(
                    xml_str,
                    output_str,
                    'Invalid XML dump string for {}, value={}, '
                    'expected={}'.format(
                        sdf_tag,
                        xml_str,
                        output_str))

            obj.reset()
            self.assertTrue(obj.is_valid(),
                            'Element {} should be valid after reset'.format(
                                sdf_tag))

            for value in INVALID_BASIC_VALUES[sdf_tag]:
                try:
                    obj.value = value
                except AssertionError:
                    pass
                else:
                    self.fail(
                        'No assertion error raised'
                        ' while setting {} for {}'.format(
                            value, sdf_tag))

    def test_pose(self):
        sdf_obj = create_sdf_element('pose')

        self.assertIsNotNone(sdf_obj, 'Invalid pose object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'pose',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, [0 for _ in range(6)],
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 1,
                         'Pose should have 1 attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Pose should have no children')
        self.assertEqual(sdf_obj.frame, '',
                         'Pose should empty frame name after reset')

        # Test setting valid numbers to the pose object
        sdf_obj.x = 1
        self.assertEqual(sdf_obj.x, 1, 'X was not set')
        sdf_obj.y = 2
        self.assertEqual(sdf_obj.y, 2, 'Y was not set')
        sdf_obj.z = 3
        self.assertEqual(sdf_obj.z, 3, 'Z was not set')
        sdf_obj.roll = 4
        self.assertEqual(sdf_obj.roll, 4, 'Roll angle was not set')
        sdf_obj.pitch = 5
        self.assertEqual(sdf_obj.pitch, 5, 'Pitch angle was not set')
        sdf_obj.yaw = 6
        self.assertEqual(sdf_obj.yaw, 6, 'Yaw angle was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2']
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.x = v
            with self.assertRaises(AssertionError):
                sdf_obj.y = v
            with self.assertRaises(AssertionError):
                sdf_obj.z = v
            with self.assertRaises(AssertionError):
                sdf_obj.roll = v
            with self.assertRaises(AssertionError):
                sdf_obj.pitch = v
            with self.assertRaises(AssertionError):
                sdf_obj.yaw = v

        for v in INVALID_POSE_VALUES:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Check invalid object value
        # Wrong vector size
        with self.assertRaises(AssertionError):
            sdf_obj.value = [0 for _ in range(5)]
        with self.assertRaises(AssertionError):
            sdf_obj.value = None
        with self.assertRaises(AssertionError):
            sdf_obj.value = ['s' for _ in range(6)]

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid pose vector, is_valid should be True')

        output_str = '0 0 0 0 0 0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.x = 1.2
        output_str = '1.2 0 0 0 0 0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'pose',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '0 0 0 0 0 0',
                         'Invalid XML element tag')

        self.assertEqual(sdf_obj.frame, '',
                         'Pose should empty frame name after reset')
        sdf_obj.frame = 'test'
        self.assertEqual(sdf_obj.frame, 'test',
                         'Pose frame name was not set')
        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<pose>0 0 0 0 0 0</pose>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_size(self):
        sdf_obj = create_sdf_element('size', [0, 0, 0])

        self.assertIsNotNone(sdf_obj, 'Invalid size object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'size',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, [0 for _ in range(3)],
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Size should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Size should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.width = 1
        self.assertEqual(sdf_obj.width, 1, 'Width was not set')
        sdf_obj.length = 2
        self.assertEqual(sdf_obj.length, 2, 'Length was not set')
        sdf_obj.height = 3
        self.assertEqual(sdf_obj.height, 3, 'Height was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.width = v
            with self.assertRaises(AssertionError):
                sdf_obj.length = v
            with self.assertRaises(AssertionError):
                sdf_obj.height = v

        # Check invalid object value
        # Wrong vector size
        with self.assertRaises(AssertionError):
            sdf_obj.value = [0 for _ in range(5)]
        with self.assertRaises(AssertionError):
            sdf_obj.value = None
        with self.assertRaises(AssertionError):
            sdf_obj.value = ['s' for _ in range(3)]

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid size vector, is_valid should be True')

        output_str = '0 0 0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.width = 1.2
        output_str = '1.2 0 0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'size',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '0 0 0',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<size>0 0 0</size>',
        #     'Invalid XML dump string, value=' + str(xml_str))

        sdf_obj = create_sdf_element('size', [0, 0])

        self.assertIsNotNone(sdf_obj, 'Invalid size object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'size',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, [0 for _ in range(2)],
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Size should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Size should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.width = 1
        self.assertEqual(sdf_obj.width, 1, 'Width was not set')
        sdf_obj.length = 2
        self.assertEqual(sdf_obj.length, 2, 'Length was not set')
        with self.assertRaises(AssertionError):
            sdf_obj.height = 3

        # Test setting invalid inputs to pose object
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.width = v
            with self.assertRaises(AssertionError):
                sdf_obj.length = v

        # Check invalid object value
        # Wrong vector size
        with self.assertRaises(AssertionError):
            sdf_obj.value = [0 for _ in range(5)]
        with self.assertRaises(AssertionError):
            sdf_obj.value = None
        with self.assertRaises(AssertionError):
            sdf_obj.value = ['s' for _ in range(2)]

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid size vector, is_valid should be True')

        output_str = '0 0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.width = 1.2
        output_str = '1.2 0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'size',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '0 0',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<size>0 0</size>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_box(self):
        sdf_obj = create_sdf_element('box')

        self.assertIsNotNone(sdf_obj, 'Invalid box object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'box',
                         'Invalid SDF block name')
        self.assertIsNone(
            sdf_obj.value,
            'Box should have no valid field <value>, only children')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Box should have no attributes')
        self.assertEqual(len(sdf_obj.children), 1,
                         'Box should have 1 child element')
        self.assertTrue(sdf_obj.is_valid(), 'Default values should be valid')

        self.assertEqual(sdf_obj.size.value, [0 for _ in range(3)],
                         'Invalid initial value {}'.format(sdf_obj.size))

        for v in INVALID_SIZE_VALUES:
            with self.assertRaises(AssertionError):
                sdf_obj.size = v

        sdf_obj.reset()

        self.assertEqual(sdf_obj.size.value, [0 for _ in range(3)],
                         'Invalid initial value after reset')

        for value in VALID_SIZE_VALUES:
            sdf_obj.size = value
            self.assertEqual(sdf_obj.size.value, value, 'Size was not set')
            sdf_obj.reset()
            self.assertEqual(sdf_obj.size.value, [0 for _ in range(3)],
                             'Invalid initial value after reset')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<box><size>0 0 0</size></box>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_length(self):
        sdf_obj = create_sdf_element('length')

        self.assertIsNotNone(sdf_obj, 'Invalid length object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'length',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 0,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Length should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Length should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 1
        self.assertEqual(sdf_obj.value, 1, 'Length was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid length, is_valid should be True')

        output_str = '0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 1.2
        output_str = '1.2'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'length',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '0',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<length>0</length>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_radius(self):
        sdf_obj = create_sdf_element('radius')

        self.assertIsNotNone(sdf_obj, 'Invalid radius object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'radius',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 0,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Radius should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Radius should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 1
        self.assertEqual(sdf_obj.value, 1, 'Radius was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid radius, is_valid should be True')

        output_str = '0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 1.2
        output_str = '1.2'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'radius',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '0',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<radius>0</radius>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_plane(self):
        sdf_obj = create_sdf_element('plane')

        self.assertIsNotNone(sdf_obj, 'Invalid plane object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'plane',
                         'Invalid SDF block name')
        self.assertIsNone(
            sdf_obj.value,
            'Plane should have no valid field <value>, only children')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Plane should have no attributes')
        self.assertEqual(len(sdf_obj.children), 2,
                         'Plane should have 1 child element')
        self.assertTrue(sdf_obj.is_valid(),
                        'Plane is not valid')

        self.assertEqual(sdf_obj.size.value, [1, 1],
                         'Invalid initial value')
        self.assertEqual(sdf_obj.normal.value, [0, 0, 1],
                         'Invalid initial value')

        invalid_values = [None, -1, [0], [0, 0, 0, 0]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.size = v
            with self.assertRaises(AssertionError):
                sdf_obj.normal = v

        sdf_obj.reset()

        self.assertEqual(sdf_obj.size.value, [1, 1],
                         'Invalid initial value')
        self.assertEqual(sdf_obj.normal.value, [0, 0, 1],
                         'Invalid initial value')
        self.assertTrue(sdf_obj.is_valid(),
                        'Sphere object is invalid')

        correct_str = '<plane><size>1 1</size>' \
            '<normal>0 0 1</normal></plane>'
        xml_str = sdf_obj.to_xml_as_str()
        self.assertEqual(
            xml_str,
            correct_str,
            'Invalid XML dump string, value=' + str(xml_str))

    def test_scale(self):
        sdf_obj = create_sdf_element('scale')

        self.assertIsNotNone(sdf_obj, 'Invalid scale object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'scale',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, [1 for _ in range(3)],
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Scale should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Scale should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = [1, 2, 3]
        self.assertEqual(sdf_obj.value, [1, 2, 3],
                         'Scale vector was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Check invalid object value
        # Wrong vector scale
        with self.assertRaises(AssertionError):
            sdf_obj.value = [0 for _ in range(5)]
        with self.assertRaises(AssertionError):
            sdf_obj.value = None
        with self.assertRaises(AssertionError):
            sdf_obj.value = ['s' for _ in range(3)]

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid pose vector, is_valid should be True')

        output_str = '1 1 1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = [1.2, 1, 1]
        output_str = '1.2 1 1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'scale',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '1 1 1',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<scale>1 1 1</scale>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_lighting(self):
        sdf_obj = create_sdf_element('lighting')

        self.assertIsNotNone(sdf_obj, 'Invalid lighting object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'lighting',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, False,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Lighting should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Lighting should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = True
        self.assertTrue(sdf_obj.value, 'lighting was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertEqual(sdf_obj.value, False,
                         'For valid lighting, is_valid should be False')

        output_str = '0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = True
        output_str = '1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'lighting',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '0',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<lighting>0</lighting>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_kinematic(self):
        sdf_obj = create_sdf_element('kinematic')

        self.assertIsNotNone(sdf_obj, 'Invalid kinematic object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'kinematic',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, False,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Kinematic should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Kinematic should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = True
        self.assertTrue(sdf_obj.value, 'Kinematic was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertEqual(sdf_obj.value, False,
                         'For valid kinematic, is_valid should be False')

        output_str = '0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = True
        output_str = '1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'kinematic',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '0',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<kinematic>0</kinematic>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_gravity(self):
        sdf_obj = create_sdf_element('gravity')

        self.assertIsNotNone(sdf_obj, 'Invalid gravity object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'gravity',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, True,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Gravity should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Gravity should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = True
        self.assertTrue(sdf_obj.value, 'Gravity was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.value,
                        'For valid gravity, is_valid should be True')

        output_str = '1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = False
        output_str = '0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'gravity',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '1',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<gravity>1</gravity>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_name(self):
        sdf_obj = create_sdf_element('name')

        self.assertIsNotNone(sdf_obj, 'Invalid name object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'name',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 'none',
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Name should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Name should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 'none'
        self.assertEqual(sdf_obj.value, 'none',
                         'Name was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, False, -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertEqual(sdf_obj.value, 'none',
                         'For valid name, is_valid should be an empty string')

        output_str = 'none'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 'test'
        output_str = 'test'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        sdf_obj.value = 'test'
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'name',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, 'test',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<name>test</name>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_uri(self):
        sdf_obj = create_sdf_element('uri')

        self.assertIsNotNone(sdf_obj, 'Invalid uri object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'uri',
                         'Invalid SDF block uri')
        self.assertEqual(sdf_obj.value, '__default__',
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'URI should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'URI should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 'none'
        self.assertEqual(sdf_obj.value, 'none',
                         'URI was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, False, -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertEqual(sdf_obj.value, '__default__',
                         'For valid uri, is_valid should be an empty string')

        output_str = '__default__'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 'test'
        output_str = 'test'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        sdf_obj.value = 'test'
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'uri',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, 'test',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<uri>test</uri>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_submesh(self):
        sdf_obj = create_sdf_element('submesh')

        self.assertIsNotNone(sdf_obj, 'Invalid submesh object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'submesh',
                         'Invalid SDF block name')
        self.assertIsNone(
            sdf_obj.value,
            'Submesh should have no valid field <value>, only children')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Submesh should have no attributes')
        self.assertEqual(len(sdf_obj.children), 2,
                         'Submesh should have 2 child elements')
        self.assertTrue(sdf_obj.is_valid(),
                        'Submesh object is invalid')

        self.assertEqual(sdf_obj.name.value, 'none',
                         'Invalid initial value')
        self.assertEqual(sdf_obj.center.value, False,
                         'Invalid initial value')

        invalid_values = [None, 'a', '2', 10, -1]
        for v in invalid_values:
            if not isinstance(v, str):
                with self.assertRaises(AssertionError):
                    sdf_obj.name = v
            if not isinstance(v, bool):
                with self.assertRaises(AssertionError):
                    sdf_obj.center = v

        sdf_obj.reset()

        self.assertEqual(sdf_obj.name.value, 'none',
                         'Invalid initial value after reset')
        self.assertEqual(sdf_obj.center.value, False,
                         'Invalid initial value after reset')
        self.assertTrue(sdf_obj.is_valid(),
                        'SubMesh object is invalid')

        sdf_obj.name = 'test'

        if sys.version_info[0] > 2:
            correct_str = '<submesh><center>0</center>' \
                '<name>test</name></submesh>'
            xml_str = sdf_obj.to_xml_as_str()
            self.assertEqual(
                xml_str,
                correct_str,
                'Invalid XML dump string, value=' + str(xml_str))

    def test_mesh(self):
        sdf_obj = create_sdf_element('mesh')

        self.assertIsNotNone(sdf_obj, 'Invalid mesh object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'mesh',
                         'Invalid SDF block name')
        self.assertIsNone(
            sdf_obj.value,
            'Mesh should have no valid field <value>, only children')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Mesh should have no attributes')
        self.assertEqual(len(sdf_obj.children), 2,
                         'Mesh should have initially 2 child elements')
        self.assertTrue(sdf_obj.is_valid(),
                        'Mesh object is invalid')

        self.assertEqual(sdf_obj.uri.value, '__default__',
                         'Invalid initial value')
        self.assertEqual(sdf_obj.scale.value, [1, 1, 1],
                         'Invalid initial value')

        invalid_values = [None, 'a', '2', 10, -1]
        for v in invalid_values:
            if not isinstance(v, str):
                with self.assertRaises(AssertionError):
                    sdf_obj.uri = v
            if not isinstance(v, bool):
                with self.assertRaises(AssertionError):
                    sdf_obj.scale = v

        sdf_obj.reset()

        self.assertEqual(sdf_obj.uri.value, '__default__',
                         'Invalid initial value after reset')
        self.assertEqual(sdf_obj.scale.value, [1 for _ in range(3)],
                         'Invalid initial value after reset')

        sdf_obj.uri = 'test'
        correct_str = '<mesh><scale>1 1 1</scale><uri>test</uri></mesh>'
        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, correct_str,
        #     'Invalid XML dump string, value=' + str(xml_str))

        # Add a submesh object
        sdf_obj.submesh = dict(name='submesh_name', center=True)
        self.assertEqual(len(sdf_obj.children), 3,
                         'Mesh should have 3 child elements')
        self.assertTrue(sdf_obj.is_valid(),
                        'Mesh object is invalid')
        if sys.version_info[0] > 2:
            correct_str = '<mesh><uri>test</uri>' \
                '<scale>1 1 1</scale>' \
                '<submesh><center>1</center>' \
                '<name>submesh_name</name></submesh>' \
                '</mesh>'
            xml_str = sdf_obj.to_xml_as_str()
            self.assertEqual(
                xml_str,
                correct_str,
                'Invalid XML dump string, value=' + str(xml_str))

        sdf_obj.reset()
        self.assertEqual(len(sdf_obj.children), 2,
                         'Mesh should have 2 child elements')
        self.assertTrue(sdf_obj.is_valid(),
                        'Mesh object is invalid')

    def test_cylinder(self):
        sdf_obj = create_sdf_element('cylinder')

        self.assertIsNotNone(sdf_obj, 'Invalid cylinder object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'cylinder',
                         'Invalid SDF block name')
        self.assertIsNone(
            sdf_obj.value,
            'Cylinder should have no valid field <value>, only children')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Cylinder should have no attributes')
        self.assertEqual(len(sdf_obj.children), 2,
                         'Cylinder should have 2 child elements')
        self.assertTrue(sdf_obj.is_valid(),
                        'Cylinder object is invalid')

        self.assertEqual(sdf_obj.length.value, 0,
                         'Invalid initial value')
        self.assertEqual(sdf_obj.radius.value, 0,
                         'Invalid initial value')

        invalid_values = [None, 'a', '2', -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.length = v
            with self.assertRaises(AssertionError):
                sdf_obj.radius = v

        sdf_obj.reset()

        self.assertEqual(sdf_obj.length.value, 0,
                         'Invalid initial value after reset')
        self.assertEqual(sdf_obj.radius.value, 0,
                         'Invalid initial value after reset')
        self.assertTrue(sdf_obj.is_valid(),
                        'Cylinder object is invalid')
        xml_str = sdf_obj.to_xml_as_str()
        if sys.version_info[0] > 2:
            correct_str = '<cylinder><radius>0</radius><length>' \
                '0</length></cylinder>'
            self.assertEqual(
                xml_str,
                correct_str,
                'Invalid XML dump string, value=' + str(xml_str))

    def test_sphere(self):
        sdf_obj = create_sdf_element('sphere')

        self.assertIsNotNone(sdf_obj, 'Invalid sphere object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'sphere',
                         'Invalid SDF block name')
        self.assertIsNone(
            sdf_obj.value,
            'Sphere should have no valid field <value>, only children')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Sphere should have no attributes')
        self.assertEqual(len(sdf_obj.children), 1,
                         'Sphere should have 1 child element')

        self.assertEqual(sdf_obj.radius.value, 0,
                         'Invalid initial value')

        invalid_values = [None, 'a', '2', -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.radius = v

        sdf_obj.reset()

        self.assertEqual(sdf_obj.radius.value, 0,
                         'Invalid initial value after reset')
        self.assertTrue(sdf_obj.is_valid(),
                        'Sphere object is invalid')

        correct_str = '<sphere><radius>0</radius></sphere>'
        xml_str = sdf_obj.to_xml_as_str()
        self.assertEqual(
            xml_str,
            correct_str,
            'Invalid XML dump string, value=' + str(xml_str))

    def test_threshold(self):
        sdf_obj = create_sdf_element('threshold')

        self.assertIsNotNone(sdf_obj, 'Invalid threshold object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'threshold',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 0,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Threshold should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Threshold should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 1
        self.assertEqual(sdf_obj.value, 1, 'Threshold was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', [5]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid threshold, is_valid should be True')

        output_str = '0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 1.2
        output_str = '1.2'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'threshold',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '0',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<threshold>0</threshold>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_height(self):
        sdf_obj = create_sdf_element('height')

        self.assertIsNotNone(sdf_obj, 'Invalid height object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'height',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 1,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Height should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Height should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 4
        self.assertEqual(sdf_obj.value, 4, 'Height was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', [5]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid height, is_valid should be True')

        output_str = '1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 1.2
        output_str = '1.2'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'height',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '1',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<height>0</height>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_geometry(self):
        sdf_obj = create_sdf_element('geometry')

        self.assertIsNotNone(sdf_obj, 'Invalid geometry object')

        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'geometry',
                         'Invalid SDF block name')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Geometry should have no attributes')
        self.assertEqual(len(sdf_obj.children), 1,
                         'Geometry should have no children')
        self.assertIsNotNone(sdf_obj.children['empty'],
                             'Invalid SDF initial block name')

        valid_test_set = dict(
            empty=dict(),
            image=VALID_IMAGE_PROPS,
            box=VALID_BOX_PROPS,
            cylinder=VALID_CYLINDER_PROPS,
            sphere=VALID_SPHERE_PROPS,
            plane=VALID_PLANE_PROPS,
            mesh=VALID_MESH_PROPS
        )

        for test in valid_test_set:
            setattr(sdf_obj, test, valid_test_set[test])
            elem = getattr(sdf_obj, test)
            for tag in valid_test_set[test]:
                self.assertFalse(
                    isinstance(
                        getattr(
                            elem, tag), float), '{} {}'.format(
                        tag, test))
                if getattr(elem, tag).has_value():
                    self.assertEqual(
                        getattr(elem, tag).value,
                        valid_test_set[test][tag],
                        'Property {} from geometry description of type {}'
                        ' was not set'.format(tag, test))

    def test_inertia(self):
        sdf_obj = create_sdf_element('inertia')

        self.assertIsNotNone(sdf_obj, 'Invalid inertia object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'inertia',
                         'Invalid SDF block name')
        self.assertIsNone(
            sdf_obj.value,
            'Inertia should have no valid field <value>, only children')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Inertia should have no attributes')
        self.assertEqual(len(sdf_obj.children), 6,
                         'Inertia should have 2 child elements')
        self.assertTrue(sdf_obj.is_valid(),
                        'Inertia object is invalid')

        invalid_values = [None, 'a', '2', [5]]
        for value in invalid_values:
            for t in sdf_obj.children.keys():
                with self.assertRaises(AssertionError):
                    setattr(sdf_obj, t, value)

        valid_values = [1, -1, 0, 2.4]
        for value in valid_values:
            for t in sdf_obj.children.keys():
                setattr(getattr(sdf_obj, t), 'value', value)
                self.assertEqual(getattr(sdf_obj, t).value, value,
                                 '{} was not set, value={}'.format(t, value))

        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'Inertia after reset returned true')

    def test_mass(self):
        sdf_obj = create_sdf_element('mass')

        self.assertIsNotNone(sdf_obj, 'Invalid mass object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'mass',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 0,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Mass should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Mass should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 1
        self.assertEqual(sdf_obj.value, 1, 'Mass was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', -1, 0]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid mass, is_valid should be true')

        sdf_obj.value = 1.2
        output_str = '1.2'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'mass',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '1.2',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<mass>0</mass>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_inertial(self):
        sdf_obj = create_sdf_element('inertial')

        self.assertIsNotNone(sdf_obj, 'Invalid mass object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'inertial',
                         'Invalid SDF block name')
        self.assertIsNone(sdf_obj.value,
                          'Value should be none')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Inertial should have no attributes')
        self.assertEqual(len(sdf_obj.children), 3,
                         'Inertial should have 3 child elements')

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid inertial, is_valid should be true')

        for value in INVALID_MASSES:
            with self.assertRaises(AssertionError):
                sdf_obj.mass = value

        valid_masses = [10, 1, 100.2]
        for value in valid_masses:
            sdf_obj.mass = value
            self.assertEqual(sdf_obj.mass.value, value, 'Mass was not set')
            self.assertTrue(sdf_obj.is_valid(), 'Inertial should be valid')
            sdf_obj.reset()

        invalid_poses = [
            None,
            [1, 2, 3],
            [-3, 'a', 0, 0, 0, 0],
            [-3, None, 0, 0, 0, 0],
            'a',
            '2'
        ]
        for value in invalid_poses:
            with self.assertRaises(AssertionError):
                sdf_obj.pose = value

        valid_poses = [
            [0 for _ in range(6)],
            [1 for _ in range(6)],
            [-1, 1, -1, 1, -1, 1]
        ]
        for value in valid_poses:
            sdf_obj.pose = value
            self.assertEqual(sdf_obj.pose.value, value, 'Pose was not set')
            self.assertTrue(sdf_obj.is_valid(), 'Inertial should be valid')
            sdf_obj.reset()

        invalid_inertias = [
            dict(ixx=1, ixy='a', ixz=1, iyy=3, iyz=0, izz=0),
            dict(ixx=None, ixy=1, ixz=1, iyy=3, iyz=0, izz=0)
        ]
        for value in invalid_inertias:
            with self.assertRaises(AssertionError):
                sdf_obj.inertia = value

        valid_inertias = [
            dict(ixx=0, ixy=0, ixz=0, iyy=0, iyz=0, izz=0),
            dict(ixx=1, ixy=2, ixz=3, iyy=4, iyz=5, izz=6)
        ]
        for value in valid_inertias:
            sdf_obj.inertia = value
            for item in value:
                self.assertEqual(getattr(sdf_obj.inertia, item).value,
                                 value[item], 'Inertia was not set')
            self.assertTrue(sdf_obj.is_valid(), 'Inertial should be valid')
            sdf_obj.reset()

    def test_link(self):
        sdf_obj = create_sdf_element('link')

        self.assertIsNotNone(sdf_obj, 'Invalid mass object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'link',
                         'Invalid SDF block name')
        self.assertIsNone(sdf_obj.value,
                          'Value should be none')
        self.assertEqual(len(sdf_obj.attributes), 1,
                         'Link should have one attribute')
        self.assertIn('name', sdf_obj.attributes,
                      'Link should have an attribute <name>')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Link should initially have no child elements')
        self.assertTrue(sdf_obj.is_valid(),
                        'Link should initially be valid')

        # Testing attribute name
        invalid_names = [None, 1, '', dict, list]
        for value in invalid_names:
            with self.assertRaises(AssertionError):
                sdf_obj.name = value

        valid_names = ['a', 'test']
        for value in valid_names:
            sdf_obj.name = value
            self.assertEqual(sdf_obj.name, value, 'Name was not set')
            self.assertTrue(sdf_obj.is_valid(), 'Link should not be valid')

        # Reseting object
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'Link should initially be valid')

        # Testing element pose
        invalid_poses = [
            None,
            [1, 2, 3],
            [-3, 'a', 0, 0, 0, 0],
            [-3, None, 0, 0, 0, 0],
            'a',
            '2'
        ]
        for value in invalid_poses:
            with self.assertRaises(AssertionError):
                sdf_obj.pose = value

        valid_poses = [
            [0 for _ in range(6)],
            [1 for _ in range(6)],
            [-1, 1, -1, 1, -1, 1]
        ]
        for value in valid_poses:
            sdf_obj.pose = value
            self.assertEqual(sdf_obj.pose.value, value, 'Pose was not set')
            self.assertTrue(sdf_obj.is_valid(), 'Link should be valid')
            sdf_obj.name = 'test'
            self.assertTrue(sdf_obj.is_valid(), 'Link should be valid')
            sdf_obj.reset()

        # Testing boolean properties
        invalid_flags = [None, 'a', 1, dict, list]
        for value in invalid_flags:
            for t in sdf_obj.children:
                if t in ['gravity', 'kinematic']:
                    with self.assertRaises(AssertionError):
                        setattr(sdf_obj, t, value)

        for value in [True, False]:
            for t in sdf_obj.children:
                if t in ['gravity', 'kinematic']:
                    setattr(sdf_obj, t, value)
                    self.assertEqual(
                        getattr(
                            sdf_obj,
                            t),
                        value,
                        '{} was not set')
                    self.assertFalse(
                        sdf_obj.is_valid(),
                        'Link should not be valid')
                    sdf_obj.name = 'test'
                    self.assertTrue(sdf_obj.is_valid(), 'Link should be valid')
                    sdf_obj.reset()

        # Test adding inertial object to link
        # Test setting mass
        sdf_obj.mass = 1
        self.assertIn('inertial', sdf_obj.children, 'Inertial was not created')
        self.assertIsNotNone(sdf_obj.inertial, 'Inertial was not created')
        self.assertIsNotNone(sdf_obj.children['inertial'],
                             'Inertial element is invalid')
        self.assertEqual(sdf_obj.mass.value, 1, 'Mass was not set')

        invalid_values = [None, 'a', '2', -1, 0]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.mass = v

        self.assertTrue(sdf_obj.is_valid(), 'Link should be valid')

        sdf_obj.mass = 1
        sdf_obj.name = 'test'
        self.assertTrue(sdf_obj.is_valid(), 'Link should be valid')
        sdf_obj.reset()

        # Testing setting center of mass
        sdf_obj.center_of_mass = [0 for _ in range(3)]
        self.assertIn('inertial', sdf_obj.children, 'Inertial was not created')
        self.assertIsNotNone(sdf_obj.inertial, 'Inertial was not created')
        self.assertIsNotNone(sdf_obj.children['inertial'],
                             'Inertial element is invalid')
        self.assertEqual(sdf_obj.center_of_mass, [0 for _ in range(3)],
                         'Center of mass was not set')

        invalid_values = [None, 'a', '2', -1, 0, [1], [1, 1]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.center_of_mass = v

        self.assertTrue(sdf_obj.is_valid(), 'Link should be valid')

        sdf_obj.center_of_mass = [0 for _ in range(3)]
        sdf_obj.name = 'test'
        self.assertTrue(sdf_obj.is_valid(), 'Link should be valid')
        sdf_obj.reset()

        # Testing setting inertial tensor
        sdf_obj.inertia = VALID_INERTIA_VALUES[0]
        self.assertIn('inertial', sdf_obj.children, 'Inertial was not created')
        self.assertIsNotNone(sdf_obj.inertial, 'Inertial was not created')
        for tag in VALID_INERTIA_VALUES[0]:
            self.assertEqual(getattr(sdf_obj.inertia, tag).value,
                             VALID_INERTIA_VALUES[0][tag],
                             'Inertial tensor was not set')

        sdf_obj.inertial = create_sdf_element('inertial')

        for v in INVALID_INERTIA_VALUES:
            try:
                sdf_obj.inertial.inertia = v
                for i in ['ixx', 'iyy', 'izz', 'ixz', 'iyz', 'ixy']:
                    self.assertNotEqual(
                        getattr(
                            sdf_obj.inertial.inertia,
                            i).value,
                        0)
            except AssertionError:
                pass
            else:
                self.fail(
                    'No assertion error raised while setting'
                    ' {} for inertia'.format(v))

        self.assertTrue(sdf_obj.is_valid(), 'Link should be valid')

        sdf_obj.inertial.inertia = VALID_INERTIA_VALUES[0]
        sdf_obj.name = 'test'
        self.assertTrue(sdf_obj.is_valid(), 'Link should be valid')
        sdf_obj.reset()

    def test_max_contacts(self):
        sdf_obj = create_sdf_element('max_contacts')

        self.assertIsNotNone(sdf_obj, 'Invalid max_contacts object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'max_contacts',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 20,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'MaxContacts should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'MaxContacts should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 1
        self.assertEqual(sdf_obj.value, 1, 'MaxContacts was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', -1, 0, 1.9]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid max_contacts, is_valid should be true')

        sdf_obj.value = 1
        output_str = '1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'max_contacts',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '1',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<max_contacts>0</max_contacts>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_collision(self):
        sdf_obj = create_sdf_element('collision', 'link')

        self.assertIsNotNone(sdf_obj, 'Invalid collision object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'collision',
                         'Invalid SDF block name')
        self.assertIsNone(sdf_obj.value,
                          'Value should be None')
        self.assertEqual(len(sdf_obj.attributes), 1,
                         'Collision should have no attributes')
        self.assertIn('name', sdf_obj.attributes,
                      'Collision must have an attribute <name>')
        self.assertEqual(len(sdf_obj.children), 1,
                         'Collision should have one child element')
        self.assertTrue(sdf_obj.is_valid(),
                        'Collision object should initially be valid')

        # Test geometry
        valid_test_set = dict(
            empty=dict(),
            image=VALID_IMAGE_PROPS,
            box=VALID_BOX_PROPS,
            cylinder=VALID_CYLINDER_PROPS,
            sphere=VALID_SPHERE_PROPS,
            plane=VALID_PLANE_PROPS,
            mesh=VALID_MESH_PROPS
        )

        for test in valid_test_set:
            setattr(sdf_obj.geometry, test, valid_test_set[test])
            elem = getattr(sdf_obj.geometry, test)
            for tag in valid_test_set[test]:
                self.assertFalse(
                    isinstance(
                        getattr(
                            elem, tag), float), '{} {}'.format(
                        tag, test))
                if getattr(elem, tag).has_value():
                    self.assertEqual(
                        getattr(elem, tag).value,
                        valid_test_set[test][tag],
                        'Property {} from geometry description of type {}'
                        ' was not set'.format(tag, test))

            self.assertTrue(sdf_obj.is_valid(),
                            'Collision should still be valid')

            sdf_obj.geometry.reset()

        # Test max contacts
        self.assertIsNone(sdf_obj.max_contacts,
                          'Max. contacts should be optional')
        sdf_obj.max_contacts = 1
        self.assertEqual(sdf_obj.max_contacts.value, 1,
                         'Max. contacts should be one')
        invalid_max_contact_values = [None, False, dict(), list(), [1, 2], -1]
        for value in invalid_max_contact_values:
            with self.assertRaises(AssertionError):
                sdf_obj.max_contacts = value

        sdf_obj.reset()

        # Test set pose
        self.assertIsNone(sdf_obj.pose,
                          'Pose should be optional')
        sdf_obj.pose = [1 for _ in range(6)]
        self.assertEqual(sdf_obj.pose.value, [1 for _ in range(6)],
                         'Pose was not set')

        for v in INVALID_POSE_VALUES:
            with self.assertRaises(AssertionError):
                sdf_obj.pose = v

        sdf_obj.reset()

    def test_script(self):
        sdf_obj = create_sdf_element('script')

        self.assertIsNotNone(sdf_obj, 'Invalid scale object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'script',
                         'Invalid SDF block name')
        self.assertIsNone(sdf_obj.value,
                          'Invalid initial value, it should be none')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Script should have no attributes')
        self.assertEqual(len(sdf_obj.children), 1,
                         'Script should have one child element')
        self.assertFalse(sdf_obj.is_valid(),
                         'Script object should initially be invalid')

        sdf_obj.name = 'test'
        self.assertEqual(sdf_obj.name.value, 'test',
                         'Script name was not set')
        self.assertEqual(len(sdf_obj.children), 1,
                         'Script should have one child element')

        self.assertFalse(sdf_obj.is_valid(),
                         'Script object should initially be invalid')
        sdf_obj.reset()
        self.assertEqual(sdf_obj.name.value, 'default', 'Name is invalid')

        sdf_obj.name = 'test'
        sdf_obj.add_uri('uri_test_0')
        self.assertEqual(len(sdf_obj.children), 2,
                         'Script should have two children element')
        self.assertEqual(sdf_obj.uris[0].value, 'uri_test_0',
                         'URI was not set')
        self.assertEqual(len(sdf_obj.uris), 1,
                         'Wrong number of URIs')
        self.assertTrue(sdf_obj.is_valid(),
                        'Script should be valid')

        sdf_obj.add_uri('uri_test_1')
        self.assertEqual(len(sdf_obj.children), 2,
                         'Script should have two children element')
        self.assertEqual(sdf_obj.uris[1].value, 'uri_test_1',
                         'URI was not set')
        self.assertEqual(len(sdf_obj.uris), 2,
                         'Wrong number of URIs')
        self.assertTrue(sdf_obj.is_valid(),
                        'Script should be valid')
        sdf_obj.reset()

    def test_normal_map(self):
        sdf_obj = create_sdf_element('normal_map')

        self.assertIsNotNone(sdf_obj, 'Invalid name object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'normal_map',
                         'Invalid SDF block normal map')
        self.assertEqual(sdf_obj.value, '',
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Normal map should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Normal map should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 'none'
        self.assertEqual(sdf_obj.value, 'none',
                         'Normal map was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, False, -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertEqual(sdf_obj.value, '',
                         'For valid name, is_valid should be an empty string')

        output_str = ''
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 'test'
        output_str = 'test'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        sdf_obj.value = 'test'
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'normal_map',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, 'test',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<name>test</name>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_shader(self):
        sdf_obj = create_sdf_element('shader')

        self.assertIsNotNone(sdf_obj, 'Invalid scale object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'shader',
                         'Invalid SDF block name')
        self.assertIsNone(sdf_obj.value,
                          'Invalid initial value, it should be none')
        self.assertEqual(len(sdf_obj.attributes), 1,
                         'Shader should have no attributes')
        self.assertEqual(len(sdf_obj.children), 1,
                         'Shader should have one child element')
        self.assertTrue(sdf_obj.is_valid(),
                        'Shader object should initially be valid')

        invalid_values = [None, 'a', '2', -1, [0], [0, 0, 0, 0], 'test']
        for value in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.type = 'value'

        self.assertTrue(sdf_obj.is_valid(),
                        'Shader should be valid')

        valid_shader_types = [
            'vertex',
            'pixel',
            'normal_map_objectspace',
            'normal_map_tangentspace'
        ]
        for value in valid_shader_types:
            sdf_obj.type = value
            self.assertEqual(sdf_obj.type, value,
                             'Type {} was not set'.format(value))

        self.assertTrue(sdf_obj.is_valid(),
                        'Shader should be valid')

        sdf_obj.normal_map = 'test'
        self.assertEqual(sdf_obj.normal_map.value, 'test',
                         'Normal map was not set')

        sdf_obj.reset()
        sdf_obj.type = 'vertex'
        self.assertEqual(sdf_obj.type, 'vertex',
                         'Type was not set')
        sdf_obj.normal_map = 'test'
        self.assertEqual(sdf_obj.normal_map.value, 'test',
                         'Normal map was not set')
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'shader',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.attrib, dict(type='vertex'),
                         'Invalid XML element tag')

    def test_specular(self):
        sdf_obj = create_sdf_element('specular')

        self.assertIsNotNone(sdf_obj, 'Invalid specular object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'specular',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, [0.1, 0.1, 0.1, 1],
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Specular should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Specular should have no children')

        # Test setting valid numbers to the pose object
        valid_values = [
            [0, 0, 0, 0],
            [0.1, 0.2, 0.4, 0.5],
            [1, 1, 1, 1]]
        for v in valid_values:
            sdf_obj.value = v
            self.assertEqual(sdf_obj.value, v,
                             'Specular vector was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', -1, [2, 1, 1, 1]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Check invalid object value
        # Wrong vector specular
        with self.assertRaises(AssertionError):
            sdf_obj.value = [0 for _ in range(5)]
        with self.assertRaises(AssertionError):
            sdf_obj.value = None
        with self.assertRaises(AssertionError):
            sdf_obj.value = ['s' for _ in range(3)]

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid pose vector, is_valid should be True')

        sdf_obj.reset()
        output_str = '0 0 0 0'
        self.assertEqual(
            sdf_obj.get_formatted_value_as_str(),
            output_str,
            'Wrong formatted output string ' +
            sdf_obj.get_formatted_value_as_str())

        sdf_obj.value = [0.2, 1, 1, 0]
        output_str = '0.2 1 1 0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'specular',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '0 0 0 0',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<specular>0 0 0 0</specular>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_material(self):
        sdf_obj = create_sdf_element('material')

        self.assertIsNotNone(sdf_obj, 'Invalid emissive object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'material',
                         'Invalid SDF block name')
        self.assertIsNone(sdf_obj.value,
                          'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Emissive should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Emissive should have no children')

        for tag in VALID_MATERIAL_PROPS:
            setattr(sdf_obj, tag, VALID_MATERIAL_PROPS[tag])
            if tag == 'shader':
                self.assertEqual(sdf_obj.shader.normal_map.value,
                                 VALID_MATERIAL_PROPS[tag]['normal_map'])
                self.assertEqual(
                    sdf_obj.shader.type,
                    VALID_MATERIAL_PROPS[tag]['attributes']['type'])
            elif tag == 'script':
                self.assertEqual(getattr(sdf_obj, tag).name.value,
                                 VALID_MATERIAL_PROPS[tag]['name'])
                self.assertEqual(getattr(sdf_obj, tag).uris[0].value,
                                 VALID_MATERIAL_PROPS[tag]['uri'])
            else:
                self.assertEqual(
                    getattr(
                        sdf_obj,
                        tag).value,
                    VALID_MATERIAL_PROPS[tag])

            self.assertTrue(sdf_obj.is_valid(), 'Material should be valid,'
                            ' setting child=' + tag)

    def test_transparency(self):
        sdf_obj = create_sdf_element('transparency')

        self.assertIsNotNone(sdf_obj, 'Invalid transparency object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'transparency',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 0,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Transparency should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Transparency should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 1
        self.assertEqual(sdf_obj.value, 1, 'Transparency was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', -1, 1.2]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid transparency, is_valid should be true')

        sdf_obj.value = 0.2
        output_str = '0.2'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'transparency',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '0.2',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<transparency>0</transparency>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_visual(self):
        sdf_obj = create_sdf_element('visual')

        self.assertIsNotNone(sdf_obj, 'Invalid visual object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'visual',
                         'Invalid SDF block name')
        self.assertIsNone(sdf_obj.value,
                          'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 1,
                         'Visual should have one attributes')
        self.assertIn('name', sdf_obj.attributes,
                      'Visual should have an attribute name')
        self.assertEqual(len(sdf_obj.children), 1,
                         'Visual should have one child element initially')

        for tag in VALID_VISUAL_PROPS:
            setattr(sdf_obj, tag, VALID_VISUAL_PROPS[tag])
            if tag in sdf_obj.attributes:
                self.assertEqual(
                    getattr(
                        sdf_obj,
                        tag),
                    VALID_VISUAL_PROPS[tag],
                    '{} was not set, value={}'.format(
                        tag,
                        getattr(
                            sdf_obj,
                            tag)))
            else:
                self.assertEqual(
                    getattr(
                        sdf_obj,
                        tag).value,
                    VALID_VISUAL_PROPS[tag],
                    '{} was not set, value={}'.format(
                        tag,
                        getattr(
                            sdf_obj,
                            tag).value))
        self.assertTrue(sdf_obj, 'Visual should be valid')

        sdf_obj.material = VALID_MATERIAL_PROPS
        self.assertTrue(sdf_obj, 'Visual should be valid')

        valid_test_set = dict(
            empty=dict(),
            image=VALID_IMAGE_PROPS,
            box=VALID_BOX_PROPS,
            cylinder=VALID_CYLINDER_PROPS,
            sphere=VALID_SPHERE_PROPS,
            plane=VALID_PLANE_PROPS,
            mesh=VALID_MESH_PROPS
        )

        for test in valid_test_set:
            setattr(sdf_obj.geometry, test, valid_test_set[test])
            elem = getattr(sdf_obj.geometry, test)
            for tag in valid_test_set[test]:
                self.assertFalse(
                    isinstance(
                        getattr(
                            elem, tag), float), '{} {}'.format(
                        tag, test))
                if getattr(elem, tag).has_value():
                    self.assertEqual(
                        getattr(elem, tag).value,
                        valid_test_set[test][tag],
                        'Property {} from geometry description of type {}'
                        ' was not set'.format(tag, test))

            self.assertTrue(sdf_obj.is_valid(),
                            'Visual should be valid')

            sdf_obj.geometry.reset()

    def test_include(self):
        sdf_obj = create_sdf_element('include')

        self.assertIsNotNone(sdf_obj, 'Invalid visual object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'include',
                         'Invalid SDF block name')
        self.assertIsNone(sdf_obj.value,
                          'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Include should have no attributes')
        self.assertIn('uri', sdf_obj.children,
                      'Visual should have an URI item')

        self.assertEqual(
            sdf_obj.uri.value, '__default__', 'Invalid initial value')
        sdf_obj.uri = 'test'
        self.assertEqual(sdf_obj.uri.value, 'test', 'Invalid value')
        self.assertTrue(sdf_obj.is_valid())

        self.assertIsNone(sdf_obj.pose, 'Pose object should not exist')
        # Test setting invalid inputs to pose object
        for v in INVALID_POSE_VALUES:
            with self.assertRaises(AssertionError):
                sdf_obj.pose = v

        sdf_obj.pose = [1 for _ in range(6)]
        self.assertEqual(
            sdf_obj.pose.value, [
                1 for _ in range(6)], 'Invalid pose')
        self.assertTrue(sdf_obj.is_valid())

        for v in INVALID_MASSES:
            with self.assertRaises(AssertionError):
                sdf_obj.static = v

        sdf_obj.reset()
        sdf_obj.uri = 'test'
        self.assertIsNone(sdf_obj.static, 'Static object should not exist')
        sdf_obj.static = True
        self.assertTrue(sdf_obj.static.value, 'Static not set')
        self.assertTrue(sdf_obj.is_valid())

        sdf_obj.reset()
        sdf_obj.uri = 'test'
        self.assertIsNone(sdf_obj.name, 'Name object should not exist')
        sdf_obj.name = 'test'
        self.assertEqual(sdf_obj.name.value, 'test', 'Name was not set')
        self.assertTrue(sdf_obj.is_valid())

    def test_max_step_size(self):
        sdf_obj = create_sdf_element('max_step_size')

        self.assertIsNotNone(sdf_obj, 'Invalid max_step_size object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'max_step_size',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 0.001,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Max. step size should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Max. step size should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 4
        self.assertEqual(sdf_obj.value, 4, 'Max. step size was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', [5]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid max_step_size, is_valid should be True')

        output_str = '0.001'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 1.2
        output_str = '1.2'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'max_step_size',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '0.001',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<max_step_size>0</max_step_size>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_real_time_factor(self):
        sdf_obj = create_sdf_element('real_time_factor')

        self.assertIsNotNone(sdf_obj, 'Invalid real_time_factor object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'real_time_factor',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 1,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Real time factor should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Real time factor should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 4
        self.assertEqual(sdf_obj.value, 4,
                         'Real time factor was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', [5]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid real_time_factor, is_valid should be True')

        output_str = '1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 1.2
        output_str = '1.2'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'real_time_factor',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '1',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<real_time_factor>0</real_time_factor>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_real_time_update_rate(self):
        sdf_obj = create_sdf_element('real_time_update_rate')

        self.assertIsNotNone(sdf_obj, 'Invalid real_time_update_rate object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'real_time_update_rate',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 1,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Real time update rate should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Real time update rate should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 4
        self.assertEqual(sdf_obj.value, 4,
                         'Real time update rate was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', [5]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(
            sdf_obj.is_valid(),
            'For valid real_time_update_rate, is_valid should be True')

        output_str = '1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 1.2
        output_str = '1.2'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'real_time_update_rate',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '1',
                         'Invalid XML element tag')

        xml_str = sdf_obj.to_xml_as_str()
        self.assertEqual(
            xml_str,
            '<real_time_update_rate>1</real_time_update_rate>',
            'Invalid XML dump string, value=' + str(xml_str))

    def test_min_step_size(self):
        sdf_obj = create_sdf_element('min_step_size')

        self.assertIsNotNone(sdf_obj, 'Invalid min_step_size object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'min_step_size',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 0.0001,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Min. step size should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Min. step size should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 4
        self.assertEqual(sdf_obj.value, 4, 'Min. step size was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', [5]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid min_step_size, is_valid should be True')

        output_str = '0.0001'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 1.2
        output_str = '1.2'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'min_step_size',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '0.0001',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<min_step_size>0</min_step_size>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_physics(self):
        sdf_obj = create_sdf_element('physics')

        self.assertIsNotNone(sdf_obj, 'Invalid physics object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'physics',
                         'Invalid SDF block name')
        self.assertIsNone(sdf_obj.value,
                          'Value should be None')
        self.assertEqual(len(sdf_obj.attributes), 3,
                         'Physics should have three attributes')
        self.assertIn('name', sdf_obj.attributes,
                      'Physics must have an attribute <name>')
        self.assertIn('default', sdf_obj.attributes,
                      'Physics must have an attribute <default>')
        self.assertIn('type', sdf_obj.attributes,
                      'Physics must have an attribute <type>')
        self.assertEqual(len(sdf_obj.children), 3,
                         'Physics should have 4 child elements')
        self.assertTrue(sdf_obj.is_valid(),
                        'Physics object should initially be valid')

        invalid_values = [None, 'a', '2', [5]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.max_step_size = v
            with self.assertRaises(AssertionError):
                sdf_obj.real_time_factor = v
            with self.assertRaises(AssertionError):
                sdf_obj.real_time_update_rate = v
            with self.assertRaises(AssertionError):
                sdf_obj.max_contacts = v

        sdf_obj.reset()
        invalid_values = [True, None, 3, 5.5, dict, list]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.name = v
            with self.assertRaises(AssertionError):
                sdf_obj.type = v

        sdf_obj.reset()
        invalid_values = [None, 3, 5.5, dict, list]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.default = v

        invalid_physics_types = ['a', 'abc', '123']
        for v in invalid_physics_types:
            with self.assertRaises(AssertionError):
                sdf_obj.type = v

        sdf_obj.reset()
        self.assertEqual(sdf_obj.name, 'default_physics')
        sdf_obj.name = 'test'
        self.assertEqual(sdf_obj.name, 'test')
        self.assertTrue(sdf_obj.is_valid())

        sdf_obj.reset()
        self.assertEqual(sdf_obj.type, 'ode')
        sdf_obj.type = 'bullet'
        self.assertEqual(sdf_obj.type, 'bullet')
        self.assertTrue(sdf_obj.is_valid())
        sdf_obj.type = 'simbody'
        self.assertEqual(sdf_obj.type, 'simbody')
        self.assertTrue(sdf_obj.is_valid())
        sdf_obj.type = 'ode'
        self.assertEqual(sdf_obj.type, 'ode')
        self.assertTrue(sdf_obj.is_valid())

        sdf_obj.reset()
        self.assertEqual(sdf_obj.max_step_size.value, 0.001)
        sdf_obj.max_step_size = 0.1
        self.assertEqual(sdf_obj.max_step_size.value, 0.1)
        self.assertTrue(sdf_obj.is_valid())

        sdf_obj.reset()
        self.assertEqual(sdf_obj.real_time_factor.value, 1)
        sdf_obj.real_time_factor = 2
        self.assertEqual(sdf_obj.real_time_factor.value, 2)
        self.assertTrue(sdf_obj.is_valid())

        sdf_obj.reset()
        self.assertEqual(sdf_obj.real_time_update_rate.value, 1000)
        sdf_obj.real_time_update_rate = 100
        self.assertEqual(sdf_obj.real_time_update_rate.value, 100)
        self.assertTrue(sdf_obj.is_valid())

        sdf_obj.reset()
        sdf_obj.max_contacts = 100
        self.assertEqual(sdf_obj.max_contacts.value, 100)
        self.assertTrue(sdf_obj.is_valid())

    def test_split_impulse(self):
        sdf_obj = create_sdf_element('split_impulse')

        self.assertIsNotNone(sdf_obj, 'Invalid split_impulse object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'split_impulse',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, True,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Split impulse should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Split impulse should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = False
        self.assertFalse(sdf_obj.value, 'Split impulse was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertEqual(sdf_obj.value, True,
                         'For valid split_impulse, is_valid should be False')

        output_str = '1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = True
        output_str = '1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'split_impulse',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '1',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<split_impulse>0</split_impulse>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_split_impulse_penetration_threshold(self):
        sdf_obj = create_sdf_element('split_impulse_penetration_threshold')

        self.assertIsNotNone(
            sdf_obj, 'Invalid split_impulse_penetration_threshold object')
        # Check default values
        self.assertEqual(
            sdf_obj.xml_element_name,
            'split_impulse_penetration_threshold',
            'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, -0.01,
                         'Invalid initial value')
        self.assertEqual(
            len(sdf_obj.attributes),
            0,
            'Split impulse penetration threshold '
            'should have no attributes')
        self.assertEqual(
            len(sdf_obj.children),
            0,
            'Split impulse penetration threshold '
            'should have no children')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', dict]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertEqual(
            sdf_obj.value,
            -0.01,
            'For valid split_impulse_penetration_threshold,'
            ' is_valid should be False')

        output_str = '-0.01'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 1
        output_str = '1.0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(
            xml_element.tag,
            'split_impulse_penetration_threshold',
            'Invalid XML element tag')
        self.assertEqual(xml_element.text, '-0.01',
                         'Invalid XML element tag')

        xml_str = sdf_obj.to_xml_as_str()
        self.assertEqual(
            xml_str,
            '<split_impulse_penetration_threshold>-0.01'
            '</split_impulse_penetration_threshold>',
            'Invalid XML dump string, value=' + str(xml_str))

    def test_constraints(self):
        sdf_obj = create_sdf_element('constraints')

        self.assertIsNotNone(sdf_obj, 'Invalid constraints object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'constraints',
                         'Invalid SDF block name')
        self.assertIsNone(sdf_obj.value,
                          'Value should be None')
        self.assertEqual(len(sdf_obj.children), 4,
                         'Constraints should have 4 child elements')
        self.assertTrue(sdf_obj.is_valid(),
                        'Constraints object should initially be valid')

        with self.assertRaises(AssertionError):
            sdf_obj.reset('test')

        sdf_obj.reset('ode')
        invalid_values = [None, 'a', '2', [5]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.cfm = v
            with self.assertRaises(AssertionError):
                sdf_obj.erp = v
            with self.assertRaises(AssertionError):
                sdf_obj.contact_surface_layer = v
            with self.assertRaises(AssertionError):
                sdf_obj.contact_max_correcting_vel = v

        self.assertIsNone(sdf_obj.split_impulse)
        self.assertIsNone(sdf_obj.split_impulse_penetration_threshold)

        sdf_obj.reset('ode')
        sdf_obj.cfm = 2
        self.assertEqual(sdf_obj.cfm.value, 2)
        sdf_obj.erp = 2
        self.assertEqual(sdf_obj.erp.value, 2)
        sdf_obj.contact_surface_layer = 0.1
        self.assertEqual(sdf_obj.contact_surface_layer.value, 0.1)
        sdf_obj.contact_max_correcting_vel = 0.1
        self.assertEqual(sdf_obj.contact_max_correcting_vel.value, 0.1)

        sdf_obj.reset('bullet')
        invalid_values = [None, 'a', '2', [5]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.cfm = v
            with self.assertRaises(AssertionError):
                sdf_obj.erp = v
            with self.assertRaises(AssertionError):
                sdf_obj.contact_surface_layer = v
            with self.assertRaises(AssertionError):
                sdf_obj.split_impulse = v
            with self.assertRaises(AssertionError):
                sdf_obj.split_impulse_penetration_threshold = v

        self.assertIsNone(sdf_obj.contact_max_correcting_vel)

        sdf_obj.reset('bullet')
        sdf_obj.cfm = 2
        self.assertEqual(sdf_obj.cfm.value, 2)
        sdf_obj.erp = 2
        self.assertEqual(sdf_obj.erp.value, 2)
        sdf_obj.contact_surface_layer = 0.1
        self.assertEqual(sdf_obj.contact_surface_layer.value, 0.1)
        sdf_obj.split_impulse = False
        self.assertEqual(sdf_obj.split_impulse.value, False)
        sdf_obj.split_impulse_penetration_threshold = 0.1
        self.assertEqual(
            sdf_obj.split_impulse_penetration_threshold.value, 0.1)

    def test_type(self):
        sdf_obj = create_sdf_element('type')

        self.assertIsNotNone(sdf_obj, 'Invalid type object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'type',
                         'Invalid SDF block type')
        self.assertEqual(sdf_obj.value, '',
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Type should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Type should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 'none'
        self.assertEqual(sdf_obj.value, 'none',
                         'Type was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, False, -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertEqual(sdf_obj.value, '',
                         'For valid type, is_valid should be an empty string')

        output_str = ''
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 'test'
        output_str = 'test'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        sdf_obj.value = 'test'
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'type',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, 'test',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<type>test</type>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_iters(self):
        sdf_obj = create_sdf_element('iters')

        self.assertIsNotNone(sdf_obj, 'Invalid iters object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'iters',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 50,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Iters should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Iters should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 4
        self.assertEqual(sdf_obj.value, 4, 'Iters was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', [5]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid iters, is_valid should be True')

        output_str = '50'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 1
        output_str = '1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'iters',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '50',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<iters>0</iters>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_sor(self):
        sdf_obj = create_sdf_element('sor')

        self.assertIsNotNone(sdf_obj, 'Invalid sor object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'sor',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 1.3,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Sor should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Sor should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 4
        self.assertEqual(sdf_obj.value, 4, 'Sor was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', [5]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid sor, is_valid should be True')

        output_str = '1.3'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 1
        output_str = '1.0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'sor',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '1.3',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<sor>0</sor>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_precon_iters(self):
        sdf_obj = create_sdf_element('precon_iters')

        self.assertIsNotNone(sdf_obj, 'Invalid precon_iters object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'precon_iters',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, 0,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'PreConIters should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'PreConIters should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = 4
        self.assertEqual(sdf_obj.value, 4, 'PreConIters was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', [5]]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertTrue(sdf_obj.is_valid(),
                        'For valid precon_iters, is_valid should be True')

        output_str = '0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = 1
        output_str = '1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'precon_iters',
                         'Invalid XML element tag')
        self.assertEqual(xml_element.text, '0',
                         'Invalid XML element tag')

        # xml_str = sdf_obj.to_xml_as_str()
        # self.assertEqual(xml_str, '<precon_iters>0</precon_iters>',
        #     'Invalid XML dump string, value=' + str(xml_str))

    def test_use_dynamic_moi_rescaling(self):
        sdf_obj = create_sdf_element('use_dynamic_moi_rescaling')

        self.assertIsNotNone(
            sdf_obj, 'Invalid use_dynamic_moi_rescaling object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'use_dynamic_moi_rescaling',
                         'Invalid SDF block name')
        self.assertEqual(sdf_obj.value, False,
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'UseDynamicMOIRescaling should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'UseDynamicMOIRescaling should have no children')

        # Test setting valid numbers to the pose object
        sdf_obj.value = True
        self.assertTrue(sdf_obj.value,
                        'UseDynamicMOIRescaling was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, 'a', '2', -1]
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertEqual(
            sdf_obj.value,
            False,
            'For valid use_dynamic_moi_rescaling, is_valid should be False')

        output_str = '0'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.value = True
        output_str = '1'
        self.assertEqual(sdf_obj.get_formatted_value_as_str(),
                         output_str,
                         'Wrong formatted output string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(xml_element.tag, 'use_dynamic_moi_rescaling',
                         'Invalid XML element tag')
        self.assertEqual(
            xml_element.text,
            '0',
            'Invalid XML element tag')

        xml_str = sdf_obj.to_xml_as_str()
        self.assertEqual(
            xml_str,
            '<use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>',
            'Invalid XML dump string, value=' + str(xml_str))

    def test_friction_model(self):
        sdf_obj = create_sdf_element('friction_model')

        self.assertIsNotNone(sdf_obj, 'Invalid friction_model object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'friction_model',
                         'Invalid SDF block friction_model')
        self.assertEqual(sdf_obj.value, 'pyramid_model',
                         'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Friction model should have no attributes')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Friction model should have no children')

        # Test setting valid numbers to the pose object
        for model_name in VALID_ODE_FRICTION_MODEL_OPTIONS:
            sdf_obj.value = model_name
            self.assertEqual(sdf_obj.value, model_name,
                             'Friction model was not set')

        # Test setting invalid inputs to pose object
        invalid_values = [None, False, -1, 'none']
        for v in invalid_values:
            with self.assertRaises(AssertionError):
                sdf_obj.value = v

        # Test reset function
        sdf_obj.reset()
        self.assertEqual(
            sdf_obj.value,
            'pyramid_model',
            'For valid friction_model, is_valid should be an empty string')

        sdf_obj.reset()
        # Test generation of XML block
        xml_element = sdf_obj.to_xml()
        self.assertEqual(
            xml_element.tag,
            'friction_model',
            'Invalid XML element tag')
        self.assertEqual(
            xml_element.text,
            'pyramid_model',
            'Invalid XML element tag')

        xml_str = sdf_obj.to_xml_as_str()
        self.assertEqual(
            xml_str,
            '<friction_model>pyramid_model</friction_model>',
            'Invalid XML dump string, value=' + str(xml_str))

    def test_solver(self):
        sdf_obj = create_sdf_element('solver')

        self.assertIsNotNone(sdf_obj, 'Invalid solver object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'solver',
                         'Invalid SDF block name')
        self.assertIsNone(sdf_obj.value,
                          'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 0,
                         'Solver should have no attributes')
        self.assertEqual(len(sdf_obj.children), 7,
                         'Solver should have 7 children for ode solver')
        self.assertTrue(sdf_obj.is_valid(), 'Solver should be valid')

        for tag in DEFAULT_ODE_SOLVER:
            self.assertEqual(
                getattr(
                    sdf_obj,
                    tag).value,
                DEFAULT_ODE_SOLVER[tag],
                '{} was not set, value={}'.format(
                    tag,
                    getattr(
                        sdf_obj,
                        tag)))

        for value in VALID_ODE_FRICTION_MODEL_OPTIONS:
            sdf_obj.friction_model = value
            self.assertEqual(sdf_obj.friction_model.value, value)
            print(sdf_obj._mode, sdf_obj.children)
            self.assertTrue(sdf_obj.is_valid(), 'Solver should be valid')

        sdf_obj.reset('bullet')
        self.assertTrue(sdf_obj.is_valid(), 'Solver should be valid')

        self.assertEqual(len(sdf_obj.children), 4,
                         'Solver should have 4 children for bullet solver')

        for tag in DEFAULT_BULLET_SOLVER:
            self.assertEqual(
                getattr(
                    sdf_obj,
                    tag).value,
                DEFAULT_BULLET_SOLVER[tag],
                '{} was not set, value={}, expected={}'.format(
                    tag,
                    getattr(
                        sdf_obj,
                        tag),
                    DEFAULT_BULLET_SOLVER[tag]))
        self.assertTrue(sdf_obj.is_valid(), 'Solver should be valid')
        with self.assertRaises(AssertionError):
            sdf_obj.reset('test')

    def test_model(self):
        sdf_obj = create_sdf_element('model')

        self.assertIsNotNone(sdf_obj, 'Invalid model object')
        # Check default values
        self.assertEqual(sdf_obj.xml_element_name, 'model',
                         'Invalid SDF block name')
        self.assertIsNone(sdf_obj.value,
                          'Invalid initial value')
        self.assertEqual(len(sdf_obj.attributes), 1,
                         'Model should have no attributes')
        self.assertIn('name', sdf_obj.attributes,
                      'Model should have an attribute name')
        self.assertEqual(len(sdf_obj.children), 0,
                         'Model should have no children')
        self.assertTrue(sdf_obj.is_valid(), 'Model should be valid')

        # Create a box model
        box = create_sdf_element('box')
        box.size = [1, 1, 1]

        # Create box link
        link = create_sdf_element('link')
        link.pose = [0 for _ in range(6)]
        # Add visual and collision geometries
        link.add_collision(name='box')
        link.collisions[0].box = box

        link.add_visual(name='box')
        link.visuals[0].box = box

        self.assertIsNone(sdf_obj.static,
                          'Static option should not be available')
        sdf_obj.static = False
        self.assertFalse(sdf_obj.static.value, 'Static should be set as false')

        self.assertIsNone(sdf_obj.allow_auto_disable,
                          'Allow auto disable option should not be available')
        sdf_obj.allow_auto_disable = False
        self.assertFalse(sdf_obj.allow_auto_disable.value,
                         'Allow auto disable flag should be set as false')

        self.assertIsNone(sdf_obj.self_collide,
                          'Self collide option should not be available')
        sdf_obj.self_collide = False
        self.assertFalse(sdf_obj.self_collide.value,
                         'Self collide flag should be set as false')

        self.assertIsNone(sdf_obj.pose,
                          'Pose option should not be available')
        sdf_obj.pose = [1 for _ in range(6)]
        self.assertEqual(sdf_obj.pose.value, [1 for _ in range(6)],
                         'Pose was not set')

        self.assertIsNone(sdf_obj.links,
                          'There should be no links')
        self.assertIsNone(sdf_obj.includes,
                          'There should be no included blocks')

        sdf_obj.add_link(name='box', link=link)

        self.assertEqual(len(sdf_obj.links), 1,
                         'There should be one link')
        self.assertEqual(sdf_obj.links[0].name, 'box',
                         'Link should be named box')

        sdf_obj.urdf = create_sdf_element('urdf')

        t = create_urdf_element('transmission')

        sdf_obj.urdf.add_transmission('t1', t)
        self.assertEqual(len(sdf_obj.urdf.transmissions), 1,
                         'Number of transmission must be 1')
        sdf_obj.urdf.add_transmission('t1', t)
        self.assertEqual(len(sdf_obj.urdf.transmissions), 1,
                         'Number of transmission must still be 1')

        t1 = sdf_obj.urdf.get_transmission_by_name('t1')
        self.assertIsNotNone(t1, 'No transmission t1 found')

        sdf_obj.urdf.add_transmission('t2', t)
        self.assertEqual(len(sdf_obj.urdf.transmissions), 2,
                         'Number of transmission must be 2')

        t2 = sdf_obj.urdf.get_transmission_by_name('t2')
        self.assertIsNotNone(t2, 'No transmission t2 found')

        sdf_obj.urdf.add_link('link1')
        self.assertEqual(len(sdf_obj.urdf.links), 1,
                         'Number of URDF links must be 1')
        sdf_obj.urdf.add_link('link1')
        self.assertEqual(len(sdf_obj.urdf.links), 1,
                         'Number of URDF links must still be 1')

        l1 = sdf_obj.urdf.get_link_by_name('link1')
        self.assertIsNotNone(l1, 'No link link1 found')

        sdf_obj.urdf.add_link('link2')
        self.assertEqual(len(sdf_obj.urdf.links), 2,
                         'Number of URDF links must be 2')

        l2 = sdf_obj.urdf.get_link_by_name('link2')
        self.assertIsNotNone(l2, 'No link link2 found')

    def test_reset_sdf(self):
        for c in get_all_sdf_element_classes():
            obj = create_sdf_element(c._NAME)
            self.assertIsNotNone(obj)
            if len(obj.modes):
                for tag in obj.modes:
                    obj.reset(mode=tag, with_optional_elements=True)
                    self.assertTrue(obj.is_valid())
            else:
                print(obj.xml_element_name)
                obj.reset(with_optional_elements=True)
                self.assertTrue(obj.is_valid())

    def test_reset_sdf_config(self):
        for c in get_all_sdf_config_element_classes():
            obj = create_sdf_config_element(c._NAME)
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
