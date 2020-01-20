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
import numpy as np
from random import random, choice, randint
import shutil
import getpass
from pcg_gazebo.utils import generate_random_string
from pcg_gazebo.simulation import load_gazebo_models, get_gazebo_model_sdf, \
    get_gazebo_model_names, get_gazebo_model_path, \
    get_gazebo_model_sdf_filenames
from pcg_gazebo.simulation import Box, Cylinder, Sphere, Joint, \
    SimulationModel
from pcg_gazebo.simulation.properties import Pose
from pcg_gazebo.parsers import parse_sdf, parse_sdf_config
from pcg_gazebo.parsers.urdf import create_urdf_element

CUR_DIR = os.path.dirname(os.path.abspath(__file__))

INVALID_SIZES = [
    dict(),
    [2, 3],
    [1 for _ in range(8)],
    list(),
    'a',
    ['a' for _ in range(3)]
]


class TestSimulationObjects(unittest.TestCase):
    def check_joint_names(self, model, joint_names):
        for joint in joint_names:
            self.assertIsNotNone(
                model.get_joint_by_name(joint),
                'Cannot find joint of name <{}>'.format(joint))

    def check_link_names(self, model, link_names):
        for link in link_names:
            self.assertIsNotNone(model.get_link_by_name(link),
                                 'Cannot find link of name <{}>'.format(link))

    def check_number_of_links(self, model, n_links):
        self.assertEqual(len(model.links), n_links,
                         'Model should have {} links'.format(n_links))

    def check_number_of_joints(self, model, n_joints):
        self.assertEqual(len(model.joints), n_joints,
                         'Model should have {} joints'.format(n_joints))

    def check_attributes(self, obj, params):
        for tag in params:
            att = getattr(obj, tag)
            if hasattr(att, 'value'):
                value = att.value
            else:
                value = att
            self.assertEqual(
                value,
                params[tag],
                'Element {} should have value {}, retrieved={}'.format(
                    tag,
                    params[tag],
                    value))

    def check_pose(self, obj, pos, rpy):
        self.assertAlmostEqual(
            np.linalg.norm(obj.pose.position - pos),
            0,
            7,
            'Position was not parsed correctly, retrieved={}, '
            'expected={}'.format(
                obj.pose.position.tolist(),
                pos))
        diff = Pose.get_transform(obj.pose.quat, Pose.rpy2quat(*rpy))

        self.assertAlmostEqual(
            np.sum(diff[0:3]), 0, 7,
            'Orientation should be {}, retrieved={}'.format(
                Pose.rpy2quat(*rpy), obj.pose.quat))

    def create_random_box(self):
        box = Box()
        size = [random() for _ in range(3)]
        name = generate_random_string(5)
        # Setting random size
        box.size = size
        box.name = name
        box.add_inertial(random())
        return box

    def test_load_test_joint_fixed(self):
        model = SimulationModel.from_gazebo_model('test_joint_fixed')

        self.assertEqual(model.name, 'test_joint_fixed',
                         'Model has the wrong name')

        PARAMS = dict(
            allow_auto_disable=1,
            static=1,
            self_collide=0
        )

        for tag in PARAMS:
            self.assertEqual(
                getattr(
                    model,
                    tag),
                PARAMS[tag],
                'Element {} should have value {}, retrieved={}'.format(
                    tag,
                    PARAMS[tag],
                    getattr(
                        model,
                        tag)))

        self.assertEqual(
            model.pose.position.tolist(),
            [0, 0, 0],
            'Model position should be [0, 0, 0], '
            'retrieved={}'.format(
                model.pose.position))
        self.assertEqual(
            model.pose.rpy,
            [0, 0, 0],
            'Model orientation should be [0, 0, 0], '
            'retrieved={}'.format(
                model.pose.rpy))

        self.check_number_of_links(model, 2)
        self.check_number_of_joints(model, 1)

        self.check_link_names(model, ['link_1', 'link_2'])
        self.check_joint_names(model, ['fixed_joint'])

        link = model.get_link_by_name('link_1')
        self.check_pose(link, [0, 0, 0], [0, 0, 0])

        self.check_attributes(
            link.inertial,
            dict(mass=10, ixx=0.1, iyy=0.1, izz=0.1, ixy=0, ixz=0, iyz=0))

        link = model.get_link_by_name('link_2')
        self.check_pose(link, [10, 0, 0], [0.2, 0.4, 0])

        self.check_attributes(
            link.inertial,
            dict(mass=20, ixx=0.2, iyy=0.2, izz=0.2, ixy=0, ixz=0, iyz=0))

    def test_load_test_static_model_object(self):
        sdf = get_gazebo_model_sdf('test_static_model')

        self.assertEqual(
            sdf._NAME,
            'sdf',
            'Parsed SDF should be of type SDF, retrieved={}'.format(
                sdf._NAME))
        self.assertEqual(len(sdf.models), 1, 'SDF should have one model')

        sdf = sdf.models[0]

        model = SimulationModel.from_sdf(sdf)

        self.check_attributes(
            model,
            dict(
                name='test_static_model',
                allow_auto_disable=1,
                static=1,
                self_collide=0
            )
        )

        self.check_pose(model, [0, 0, 0], [0, 0, 0])

        self.check_number_of_links(model, 1)
        self.check_number_of_joints(model, 0)
        self.check_link_names(model, ['body'])

        link = model.get_link_by_name('body')

        self.check_attributes(
            link,
            dict(
                name='body',
                self_collide=0,
                kinematic=0
            )
        )

        self.assertEqual(len(link.visuals), 1,
                         'Link should have one visual element')
        self.assertIsNotNone(link.get_visual_by_name('body_visual'),
                             'Cannot find visual of name <body_visual>')

        visual = link.get_visual_by_name('body_visual')

        self.check_attributes(
            visual,
            dict(
                name='body_visual',
                cast_shadows=1,
                transparency=0
            )
        )

        self.assertEqual(visual.geometry.get_type(), 'box',
                         'Visual geometry should be a box')

        self.check_pose(visual, [0, 0.4, 0], [0, -0.7, 0])

        BOX_SIZE = [1, 2, 3]
        self.assertEqual(
            visual.geometry.get_param('size'),
            BOX_SIZE,
            'Visual geometry box should have size {}, retrieved={}'.format(
                BOX_SIZE,
                visual.geometry.get_param('size')))

        self.assertEqual(len(link.collisions), 1,
                         'Link should have one collision element')
        self.assertIsNotNone(link.get_collision_by_name('body_collision'),
                             'Cannot find collision of name <body_collision>')

        collision = link.get_collision_by_name('body_collision')

        self.check_attributes(
            collision,
            dict(
                name='body_collision',
                max_contacts=10
            )
        )

        self.assertEqual(collision.geometry.get_type(), 'cylinder',
                         'Collision geometry should be a cylinder')
        CYL_RADIUS = 2.4
        CYL_LENGTH = 5.6
        self.assertEqual(
            collision.geometry.get_param('radius'),
            CYL_RADIUS,
            'Collision cylinder radius is wrong, '
            'retrieved={}, expected={}'.format(
                collision.geometry.get_param('radius'),
                CYL_RADIUS))
        self.assertEqual(
            collision.geometry.get_param('length'),
            CYL_LENGTH,
            'Collision cylinder length is wrong, '
            'retrieved={}, expected={}'.format(
                collision.geometry.get_param('length'),
                CYL_LENGTH))

        self.check_pose(collision, [0.6, 1.2, 0.05], [0, 0, 0])

        PARAMS = dict(
            mu=1,
            mu2=0.5,
            slip1=0,
            slip2=0,
            fdir1=[0, 0, 0]
        )

        for tag in PARAMS:
            self.assertEqual(
                collision.get_ode_friction_param(tag),
                PARAMS[tag],
                'Element {} should have value {}, retrieved={}'.format(
                    tag,
                    PARAMS[tag],
                    collision.get_ode_friction_param(tag)))

        PARAMS = dict(
            soft_cfm=0,
            soft_erp=None,
            kp=None,
            kd=None,
            max_vel=None,
            min_depth=None
        )

        for tag in PARAMS:
            self.assertEqual(
                collision.get_ode_contact_param(tag),
                PARAMS[tag],
                'Element {} should have value {}, retrieved={}'.format(
                    tag,
                    PARAMS[tag],
                    collision.get_ode_contact_param(tag)))

    def test_load_test_static_model_sdf(self):
        sdf = get_gazebo_model_sdf('test_static_model')

        self.assertEqual(
            sdf._NAME,
            'sdf',
            'Parsed SDF should be of type SDF, retrieved={}'.format(
                sdf._NAME))
        self.assertEqual(len(sdf.models), 1, 'SDF should have one model')

        sdf = sdf.models[0]

        self.assertEqual(sdf.name, 'test_static_model',
                         'Model has the wrong name')

        self.check_attributes(
            sdf,
            dict(
                allow_auto_disable=1,
                static=1,
                self_collide=0,
                pose=[0 for _ in range(6)]
            )
        )

        # Parsing links
        self.assertEqual(len(sdf.links), 1, 'Model should have one link')

        sdf = sdf.links[0]
        self.assertEqual(sdf.name, 'body', 'Link should be named <body>')

        PARAMS = dict(
            name='body',
            pose=[1, 0, 3.5, 0, 0, 0],
            self_collide=False,
            kinematic=False
        )

        self.check_attributes(sdf, PARAMS)

        # Parsing visual
        self.assertEqual(len(sdf.visuals), 1,
                         'Link should have one visual element')
        self.assertEqual(
            sdf.visuals[0].name,
            'body_visual',
            'Visual must have name body_visual')
        self.assertIsNotNone(sdf.visuals[0].geometry.box,
                             'Visual has no box element')

        BOX_SIZE = [1, 2, 3]
        self.assertEqual(
            sdf.visuals[0].geometry.box.size.value,
            BOX_SIZE,
            'Box has the wrong size, retrieved={}, expected={}'.format(
                sdf.visuals[0].geometry.box.size.value,
                BOX_SIZE))

        POSE_VISUAL = [0, 0.4, 0, 0, -0.7, 0]
        self.assertEqual(
            sdf.visuals[0].pose.value,
            POSE_VISUAL,
            'Visual pose is wrong, retrieved={}, expected={}'.format(
                sdf.visuals[0].pose.value,
                POSE_VISUAL))

        self.assertTrue(sdf.visuals[0].cast_shadows.value,
                        'cast_shadows should be enabled')
        self.assertFalse(sdf.visuals[0].transparency.value,
                         'transparency should be disabled')

        # Parsing collision
        self.assertEqual(len(sdf.collisions), 1,
                         'Link should have one collision element')
        self.assertEqual(sdf.collisions[0].name, 'body_collision',
                         'Collision must have name body_collision')
        self.assertIsNotNone(sdf.collisions[0].geometry.cylinder,
                             'Collision has no cylinder element')

        CYL_RADIUS = 2.4
        CYL_LENGTH = 5.6
        self.assertEqual(
            sdf.collisions[0].geometry.cylinder.radius.value,
            CYL_RADIUS,
            'Collision cylinder radius is wrong, '
            'retrieved={}, expected={}'.format(
                sdf.collisions[0].geometry.cylinder.radius.value,
                CYL_RADIUS))
        self.assertEqual(
            sdf.collisions[0].geometry.cylinder.length.value,
            CYL_LENGTH,
            'Collision cylinder length is wrong, '
            'retrieved={}, expected={}'.format(
                sdf.collisions[0].geometry.cylinder.length.value,
                CYL_LENGTH))

        self.assertEqual(sdf.collisions[0].max_contacts.value, 10,
                         'Max. number of contacts is wrong')

        POSE_COLLISION = [0.6, 1.2, 0.05, 0, 0, 0]
        self.assertEqual(
            sdf.collisions[0].pose.value,
            POSE_COLLISION,
            'Collision pose is wrong, retrieved={}, expected={}'.format(
                sdf.collisions[0].pose.value,
                POSE_COLLISION))

        MU = 1
        MU2 = 0.5
        self.assertEqual(
            sdf.collisions[0].surface.friction.ode.mu.value,
            MU,
            'Collision mu is wrong, retrieved={}, expected={}'.format(
                sdf.collisions[0].surface.friction.ode.mu.value,
                MU))
        self.assertEqual(
            sdf.collisions[0].surface.friction.ode.mu2.value,
            MU2,
            'Collision mu2 is wrong, retrieved={}, expected={}'.format(
                sdf.collisions[0].surface.friction.ode.mu2.value,
                MU2))

        SLIP = 0.5
        self.assertEqual(
            sdf.collisions[0].surface.friction.torsional.ode.slip.value,
            SLIP,
            'Collision torsional slip is wrong, retrieved={}, '
            'expected={}'.format(
                sdf.collisions[0].surface.friction.torsional.ode.slip.value,
                SLIP))

    def test_load_gazebo_models(self):
        load_gazebo_models()

        self.assertGreater(len(get_gazebo_model_names()), 1,
                           'There should be at least one model in the list')

        root_test_model_path = os.path.join(
            CUR_DIR, 'gazebo_models')

        LIST_TEST_MODELS = dict(
            test_static_model=dict(
                path=os.path.join(root_test_model_path, 'test_static_model'),
                sdf=['model.sdf']))

        for test_model in LIST_TEST_MODELS:
            self.assertIn(test_model, get_gazebo_model_names(),
                          'Model {} not found in list of Gazebo models')

            self.assertEqual(
                get_gazebo_model_path(test_model),
                LIST_TEST_MODELS[test_model]['path'],
                'Invalid path returned for model {}'.format(
                    test_model))

            self.assertEqual(
                get_gazebo_model_sdf_filenames(test_model),
                LIST_TEST_MODELS[test_model]['sdf'],
                'Invalid list of SDF files returned '
                'for model {}'.format(test_model))

        INVALID_MODELS = [generate_random_string(4) for _ in range(10)]

        for model_name in INVALID_MODELS:
            self.assertIsNone(
                get_gazebo_model_path(model_name),
                'Function should return None for non-existent model')

    def test_model(self):
        pass

    def test_joint(self):
        # Test assertion error at joint constructor with missing arguments
        joint_args = [
            dict(
                name=None,
                parent=generate_random_string(3),
                child=generate_random_string(3)),
            dict(
                name=generate_random_string(3),
                parent=None,
                child=generate_random_string(3)),
            dict(
                name=generate_random_string(3),
                parent=generate_random_string(3),
                child=None),
            dict(
                name=generate_random_string(3),
                parent=generate_random_string(3),
                child=generate_random_string(3),
                joint_type='abc')]
        for item in joint_args:
            with self.assertRaises(AssertionError):
                joint = Joint(**item)

        joint_types = ['revolute', 'revolute2', 'gearbox', 'prismatic',
                       'ball', 'screw', 'universal', 'fixed']

        N_TESTS = 10
        valid_inputs = [dict(
            name=generate_random_string(3),
            parent=generate_random_string(3),
            child=generate_random_string(3),
            joint_type=choice(joint_types)) for _ in range(N_TESTS)]

        for joint_input in valid_inputs:
            joint = Joint(**joint_input)
            self.assertEqual(
                joint.name,
                joint_input['name'],
                'Joint name was not set, expected={}, '
                'retrieved={}'.format(
                    joint_input['name'],
                    joint.name))
            self.assertEqual(
                joint.parent,
                joint_input['parent'],
                'Joint parent name was not set, expected={}, '
                'retrieved={}'.format(
                    joint_input['parent'],
                    joint.parent))
            self.assertEqual(
                joint.child,
                joint_input['child'],
                'Joint child name was not set, expected={}, '
                'retrieved={}'.format(
                    joint_input['child'],
                    joint.child))
            self.assertEqual(
                joint.type,
                joint_input['joint_type'],
                'Joint type was not set, expected={}, '
                'retrieved={}'.format(
                    joint_input['joint_type'],
                    joint.type))

        # Testing parameter setting for fixed joints
        joint = Joint(
            name=generate_random_string(3),
            parent=generate_random_string(3),
            child=generate_random_string(3),
            joint_type='fixed')

        self.assertFalse(joint.set_axis_xyz([0, 0, 1]),
                         'Axis vector should be ignored for fixed joints')
        self.assertFalse(joint.set_axis_limits(
            lower=-1 * random(),
            upper=random()), 'Fixed joints have no limits')
        self.assertFalse(joint.set_axis_dynamics(
            damping=random(), friction=random()),
            'Fixed joints have no dynamic parameters')

        # TODO Add tests for universal and revolute2 joints
        # TODO Add tests for revolute and prismatic joints
        # TODO Add SDF parsing tests

    def test_sphere_object(self):
        sphere = Sphere()
        self.assertEqual(sphere.radius, 1, 'Default radius must be 1')

        N_TESTS = 10
        valid_radius = [random() for _ in range(10)]
        names = [generate_random_string(5) for _ in range(N_TESTS)]

        for s_radius, s_name in zip(valid_radius, names):
            sphere = Sphere(name=s_name, radius=s_radius)
            self.assertEqual(
                sphere.radius,
                s_radius,
                'Radius should be {}'.format(s_radius))
            self.assertEqual(
                sphere.name,
                s_name,
                'Name should be {}'.format(s_name))

            # Test setting attributes
            sphere = Sphere()
            sphere.name = s_name
            sphere.radius = s_radius
            self.assertEqual(
                sphere.radius,
                s_radius,
                'Radius should be {}'.format(s_radius))
            self.assertEqual(
                sphere.name,
                s_name,
                'Name should be {}'.format(s_name))

            sdf = sphere.to_sdf('sphere')
            self.assertEqual(
                sdf.radius.value,
                s_radius,
                'SDF converted radius is invalid')

        # Test setting different pose vectors (position + quaternion)
        valid_positions = [[random() for _ in range(3)]
                           for _ in range(N_TESTS)]

        for pos in valid_positions:
            sphere = Sphere()
            sphere.pose = pos + [0.0 for _ in range(3)]

            self.assertEqual(
                sphere.pose.position.tolist(),
                pos,
                'Sphere position should be {}, retrieved={}'.format(
                    pos,
                    sphere.pose.position))

        sdf_types = [
            'sphere',
            'geometry',
            'collision',
            'visual',
            'link',
            'model']
        for t in sdf_types:
            sphere = Sphere()
            radius = random()
            name = generate_random_string(5)
            # Setting random size
            sphere.radius = radius
            sphere.name = name
            sdf = sphere.to_sdf(t)

            self.assertEqual(
                sdf._NAME, t, 'Wrong SDF type in request for {}'.format(t))
            self.assertTrue(
                sdf.is_valid(),
                'Invalid SDF structure was returned')

            sdf_radius = None
            if sdf._NAME == 'sphere':
                sdf_radius = sdf.radius.value
            elif sdf._NAME == 'geometry':
                sdf_radius = sdf.sphere.radius.value
            elif sdf._NAME in ['collision', 'visual']:
                sdf_radius = sdf.geometry.sphere.radius.value

            if sdf_radius is not None:
                self.assertEqual(
                    sdf_radius, radius, 'SDF element has the wrong radius,'
                    ' expected={}, received={}'.format(
                        radius, sdf_radius))

            if sdf._NAME in ['link', 'model']:
                self.assertEqual(
                    sdf.name,
                    name,
                    'Name property for {} was not set, expected={}, '
                    'retrieved={}'.format(
                        sdf._NAME,
                        name,
                        sdf.name))

    def test_cylinder_object(self):
        cylinder = Cylinder()
        self.assertEqual(cylinder.radius, 1, 'Default radius must be 1')
        self.assertEqual(cylinder.length, 1, 'Default length must be 1')

        N_TESTS = 10
        valid_sizes = [[random(), random()] for _ in range(N_TESTS)]
        names = [generate_random_string(5) for _ in range(N_TESTS)]

        for cyl_size, cyl_name in zip(valid_sizes, names):
            # Testing setting cylinder size on constructor
            cylinder = Cylinder(
                name=cyl_name,
                radius=cyl_size[0],
                length=cyl_size[1])
            self.assertEqual(
                cylinder.radius,
                cyl_size[0],
                'Radius should be {}'.format(
                    cyl_size[0]))
            self.assertEqual(
                cylinder.length,
                cyl_size[1],
                'Length should be {}'.format(
                    cyl_size[1]))
            self.assertEqual(
                cylinder.name,
                cyl_name,
                'Name should be {}'.format(cyl_name))

            # Test setting attribute
            cylinder = Cylinder()
            cylinder.name = cyl_name
            cylinder.radius = cyl_size[0]
            cylinder.length = cyl_size[1]
            self.assertEqual(
                cylinder.radius,
                cyl_size[0],
                'Radius should be {}'.format(
                    cyl_size[0]))
            self.assertEqual(
                cylinder.length,
                cyl_size[1],
                'Length should be {}'.format(
                    cyl_size[1]))
            self.assertEqual(
                cylinder.name,
                cyl_name,
                'Name should be {}'.format(cyl_name))

            sdf = cylinder.to_sdf('cylinder')
            self.assertEqual(
                sdf.radius.value,
                cyl_size[0],
                'SDF converted radius is invalid')
            self.assertEqual(
                sdf.length.value,
                cyl_size[1],
                'SDF converted length is invalid')

        # Test setting different pose vectors (position + Euler angles)
        valid_poses = \
            [[random() for _ in range(3)] +
             [random() * 2 * np.pi for _ in range(3)] for _ in range(N_TESTS)]
        for pose in valid_poses:
            cylinder = Cylinder()
            cylinder.pose = pose
            self.assertEqual(
                cylinder.pose.position.tolist(),
                pose[0:3],
                'Box position should be {}, retrieved={}'.format(
                    pose[0:3], cylinder.pose.position))

            ref_q = Pose.rpy2quat(*pose[3::])
            q = cylinder.pose.quat
            diff = Pose.get_transform(ref_q, q)

            self.assertAlmostEqual(
                np.sum(diff[0:3]), 0, 7,
                'Box orientation element rpy should be {}, '
                'retrieved={}'.format(pose[3::], q))

        # Test setting different pose vectors (position + quaternion)
        valid_positions = [[random() for _ in range(3)]
                           for _ in range(N_TESTS)]
        valid_quat = [Pose.random_orientation().quat for _ in range(N_TESTS)]

        for pos, quat in zip(valid_positions, valid_quat):
            cylinder = Cylinder()
            cylinder.pose = pos + list(quat)

            self.assertEqual(
                cylinder.pose.position.tolist(),
                pos,
                'Box position should be {}, retrieved={}'.format(
                    pos,
                    cylinder.pose.position))

            ref_q = list(quat)
            q = list(cylinder.pose.quat)

            self.assertEqual(
                ref_q,
                q,
                'Quaternion was assigned incorrectly, '
                'expected={}, retrieved={}'.format(
                    ref_q,
                    q))

        sdf_types = [
            'cylinder',
            'geometry',
            'collision',
            'visual',
            'link',
            'model']
        for t in sdf_types:
            cylinder = Cylinder()
            size = [random(), random()]
            name = generate_random_string(5)
            # Setting random size
            cylinder.radius = size[0]
            cylinder.length = size[1]
            cylinder.name = name
            sdf = cylinder.to_sdf(t)

            self.assertEqual(
                sdf._NAME, t, 'Wrong SDF type in request for {}'.format(t))
            self.assertTrue(
                sdf.is_valid(),
                'Invalid SDF structure was returned')

            sdf_radius = None
            sdf_length = None
            if sdf._NAME == 'cylinder':
                sdf_radius = sdf.radius.value
                sdf_length = sdf.length.value
            elif sdf._NAME == 'geometry':
                sdf_radius = sdf.cylinder.radius.value
                sdf_length = sdf.cylinder.length.value
            elif sdf._NAME in ['collision', 'visual']:
                sdf_radius = sdf.geometry.cylinder.radius.value
                sdf_length = sdf.geometry.cylinder.length.value

            if sdf_radius is not None and sdf_length is not None:
                self.assertEqual(
                    sdf_radius, size[0], 'SDF element has the wrong radius,'
                    ' expected={}, received={}'.format(
                        size[0], sdf_radius))
                self.assertEqual(
                    sdf_length, size[1], 'SDF element has the wrong length,'
                    ' expected={}, received={}'.format(
                        size[1], sdf_length))

            if sdf._NAME in ['link', 'model']:
                self.assertEqual(
                    sdf.name,
                    name,
                    'Name property for {} was not set, expected={}, '
                    'retrieved={}'.format(
                        sdf._NAME,
                        name,
                        sdf.name))

    def test_box_object(self):
        box = Box()
        self.assertEqual(box.size, [1, 1, 1], 'Default size should be 1, 1, 1')

        # Test setting different box sizes
        N_TESTS = 10
        valid_sizes = [[random() for _ in range(3)] for _ in range(N_TESTS)]
        names = [generate_random_string(5) for _ in range(N_TESTS)]

        for box_size, box_name in zip(valid_sizes, names):
            # Test setting box size on constructor
            box = Box(name=box_name, size=box_size)
            self.assertEqual(
                box.size,
                box_size,
                'Size should be {}'.format(box_size))
            self.assertEqual(
                box.name,
                box_name,
                'Name should be {}'.format(box_name))

            # Test setting the attribute
            box = Box()
            box.name = box_name
            box.size = box_size
            self.assertEqual(
                box.size,
                box_size,
                'Size should be {}'.format(box_size))
            self.assertEqual(
                box.name,
                box_name,
                'Name should be {}'.format(box_name))

            sdf = box.to_sdf('box')
            self.assertEqual(
                sdf.size.value,
                box_size,
                'SDF converted size is invalid')

        # Test setting different pose vectors (position + Euler angles)
        valid_poses = \
            [[random() for _ in range(3)] +
             [random() * 2 * np.pi for _ in range(3)] for _ in range(N_TESTS)]
        for pose in valid_poses:
            box = Box()
            box.pose = pose
            self.assertEqual(
                box.pose.position.tolist(),
                pose[0:3],
                'Box position should be {}, retrieved={}'.format(
                    pose[0:3], box.pose.position))

            ref_q = Pose.rpy2quat(*pose[3::])
            q = box.pose.quat
            diff = Pose.get_transform(ref_q, q)

            self.assertAlmostEqual(np.sum(
                diff[0:3]), 0, 7, 'Box orientation element rpy '
                'should be {}, retrieved={}'.format(pose[3::], q))

        # Test setting different pose vectors (position + quaternion)
        valid_positions = [[random() for _ in range(3)]
                           for _ in range(N_TESTS)]
        valid_quat = [Pose.random_orientation().quat for _ in range(N_TESTS)]

        for pos, quat in zip(valid_positions, valid_quat):
            box = Box()
            box.pose = pos + list(quat)

            self.assertEqual(
                box.pose.position.tolist(),
                pos,
                'Box position should be {}, retrieved={}'.format(
                    pos,
                    box.pose.position))

            ref_q = list(quat)
            q = list(box.pose.quat)

            self.assertEqual(
                ref_q,
                q,
                'Quaternion was assigned incorrectly, '
                'expected={}, retrieved={}'.format(
                    ref_q,
                    q))

        for value in INVALID_SIZES:
            with self.assertRaises(AssertionError):
                box = Box(size=value)

        invalid_sdf_types = ['a', dict(), None, True, 1]
        for t in invalid_sdf_types:
            with self.assertRaises(AssertionError):
                box.to_sdf(t)

        sdf_types = ['box', 'geometry', 'collision', 'visual', 'link', 'model']
        for t in sdf_types:
            box = Box()
            size = [random() for _ in range(3)]
            name = generate_random_string(5)
            # Setting random size
            box.size = size
            box.name = name

            sdf = box.to_sdf(t)

            self.assertEqual(
                sdf._NAME, t, 'Wrong SDF type in request for {}'.format(t))
            self.assertTrue(
                sdf.is_valid(),
                'Invalid SDF structure was returned')

            sdf_size = None
            if sdf._NAME == 'box':
                sdf_size = sdf.size.value
            elif sdf._NAME == 'geometry':
                sdf_size = sdf.box.size.value
            elif sdf._NAME in ['collision', 'visual']:
                sdf_size = sdf.geometry.box.size.value

            if sdf_size is not None:
                self.assertEqual(
                    sdf_size, size, 'SDF element has the wrong size vector,'
                    ' expected={}, received={}'.format(
                        size, sdf_size))

            if sdf._NAME in ['link', 'model']:
                self.assertEqual(
                    sdf.name,
                    name,
                    'Name property for {} was not set, expected={}, '
                    'retrieved={}'.format(
                        sdf._NAME,
                        name,
                        sdf.name))

    def test_export_model_to_sdf(self):
        box = self.create_random_box()

        sdf = box.to_sdf('model')
        filename = '/tmp/box.sdf'
        sdf.export_xml(filename, version='1.6')

        self.assertTrue(os.path.isfile(filename))

        # Parse the exported file
        parsed_sdf = parse_sdf(filename)
        self.assertIsNotNone(parsed_sdf)
        self.assertEqual(sdf, parsed_sdf)

    def test_export_model_to_gazebo_model(self):
        box = self.create_random_box()
        model = SimulationModel.from_sdf(box.to_sdf())

        self.assertIsNotNone(model)

        # Export Gazebo model with default parameters
        model.to_gazebo_model()
        default_dir = os.path.join(
            os.path.expanduser('~'), '.gazebo', 'models')
        model_dir = os.path.join(default_dir, box.name)
        # Check if all model files were created
        self.assertTrue(os.path.isdir(model_dir))
        self.assertTrue(
            os.path.isfile(
                os.path.join(
                    model_dir,
                    'model.config')))
        self.assertTrue(os.path.isfile(os.path.join(model_dir, 'model.sdf')))

        # Parse model config file
        sdf_config = parse_sdf_config(os.path.join(model_dir, 'model.config'))
        self.assertIsNotNone(sdf_config)

        # Get name of current user, used in default model metadata input
        username = getpass.getuser()

        self.assertEqual(sdf_config.name.value, box.name)
        self.assertEqual(len(sdf_config.authors), 1)
        self.assertEqual(sdf_config.authors[0].name.value, username)
        self.assertEqual(
            sdf_config.authors[0].email.value,
            '{}@email.com'.format(username))
        self.assertEqual(sdf_config.description.value, '')
        self.assertEqual(sdf_config.version.value, '1.6')
        self.assertEqual(len(sdf_config.sdfs), 1)
        self.assertEqual(sdf_config.sdfs[0].value, 'model.sdf')
        self.assertEqual(sdf_config.sdfs[0].version, '1.6')

        sdf = parse_sdf(os.path.join(model_dir, 'model.sdf'))
        self.assertIsNotNone(sdf)
        self.assertEqual(sdf.xml_element_name, 'sdf')
        self.assertIsNotNone(sdf.models)
        self.assertEqual(len(sdf.models), 1)

        # Export Gazebo model with default parameters
        author = generate_random_string(5)
        email = generate_random_string(5)
        description = generate_random_string(10)
        model_metaname = generate_random_string(10)

        model.to_gazebo_model(
            author=author,
            email=email,
            description=description,
            model_metaname=model_metaname,
            overwrite=True
        )

        # Check if all model files were created
        self.assertTrue(os.path.isdir(model_dir))
        self.assertTrue(
            os.path.isfile(
                os.path.join(
                    model_dir,
                    'model.config')))
        self.assertTrue(os.path.isfile(os.path.join(model_dir, 'model.sdf')))

        # Parse model config file
        sdf_config = parse_sdf_config(os.path.join(model_dir, 'model.config'))
        self.assertIsNotNone(sdf_config)

        self.assertEqual(sdf_config.name.value, model_metaname)
        self.assertEqual(len(sdf_config.authors), 1)
        self.assertEqual(sdf_config.authors[0].name.value, author)
        self.assertEqual(sdf_config.authors[0].email.value, email)
        self.assertEqual(sdf_config.description.value, description)
        self.assertEqual(sdf_config.version.value, '1.6')
        self.assertEqual(len(sdf_config.sdfs), 1)
        self.assertEqual(sdf_config.sdfs[0].value, 'model.sdf')
        self.assertEqual(sdf_config.sdfs[0].version, '1.6')

        # Parse the SDF file
        sdf = parse_sdf(os.path.join(model_dir, 'model.sdf'))
        self.assertIsNotNone(sdf)
        self.assertEqual(sdf.xml_element_name, 'sdf')
        self.assertIsNotNone(sdf.models)
        self.assertEqual(len(sdf.models), 1)

        # Delete generated Gazebo model directory
        shutil.rmtree(model_dir)

    def test_convert_from_urdf(self):
        # Test correct conversion of joint relative
        # poses into absolute link poses

        # Test random sets of poses
        pose_sets = [
            [Pose.random() for _ in range(5)],
            [Pose.random_position() for _ in range(5)],
            [Pose.random_orientation() for _ in range(5)]
        ]

        for poses in pose_sets:
            urdf = create_urdf_element('robot')
            # Create links
            for i in range(len(poses) + 1):
                urdf.add_link('link_{}'.format(i))

            for i in range(len(poses)):
                joint_urdf = create_urdf_element('joint')
                joint_urdf.parent.link = 'link_{}'.format(i)
                joint_urdf.child.link = 'link_{}'.format(i + 1)
                joint_urdf.origin = poses[i].to_urdf()

                urdf.add_joint('joint_{}'.format(i), joint_urdf)

            model = SimulationModel.from_urdf(urdf)

            self.assertIsNotNone(model)

            cur_pose = None
            for i in range(len(poses) + 1):
                link = model.get_link_by_name('link_{}'.format(i))
                self.assertIsNotNone(link)

                if i == 0:
                    self.assertEqual(
                        link.pose, Pose(
                            pos=[
                                0, 0, 0], rot=[
                                0, 0, 0]))
                else:
                    if cur_pose is None:
                        cur_pose = poses[i - 1]
                    else:
                        cur_pose = cur_pose + poses[i - 1]
                    self.check_pose(link, cur_pose.position, cur_pose.rpy)

    def test_star_urdf_structure(self):
        urdf = create_urdf_element('robot')

        # Add origin link
        i = 0
        urdf.add_link('link_{}'.format(i))
        i += 1

        for _ in range(3):
            for j in range(randint(3, 5)):
                urdf.add_link('link_{}'.format(i))

                joint_urdf = create_urdf_element('joint')
                if j == 0:
                    joint_urdf.parent.link = 'link_{}'.format(0)
                else:
                    joint_urdf.parent.link = 'link_{}'.format(i - 1)
                joint_urdf.child.link = 'link_{}'.format(i)
                joint_urdf.origin = Pose.random().to_urdf()

                urdf.add_joint('joint_{}'.format(i), joint_urdf)
                i += 1

        model = SimulationModel.from_urdf(urdf)
        self.assertIsNotNone(model)

    def test_invalid_disconnected_robot_graph(self):
        urdf = create_urdf_element('robot')
        for i in range(3):
            urdf.add_link('link_{}'.format(i))

        with self.assertRaises(ValueError):
            SimulationModel.from_urdf(urdf)


if __name__ == '__main__':
    unittest.main()
