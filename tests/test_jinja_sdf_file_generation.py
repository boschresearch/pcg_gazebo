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
import random
import string
import numpy as np
from pcg_gazebo.utils import process_jinja_template
from pcg_gazebo.parsers import parse_sdf

CUR_DIR = os.path.dirname(os.path.abspath(__file__))


def get_random_string(size=3):
    return ''.join(random.choice(string.ascii_letters) for i in range(size))


def generate_sdf(test_case, params):
    xml = process_jinja_template(os.path.join(
        CUR_DIR, 'jinja_sdf', '{}.jinja'.format(test_case)), params)
    return parse_sdf(xml)


class TestJinjaSDFFileGeneration(unittest.TestCase):
    def test_jinja_ode_physics(self):
        test_case = 'physics_ode'

        params = dict(
            label=get_random_string(3),
            engine='ode',
            max_step_size=random.random() * 0.1,
            real_time_factor=random.randint(1, 5),
            real_time_update_rate=random.randint(100, 1000),
            max_contacts=random.randint(1, 20),
            ode_solver_type=random.choice(['quick', 'world']),
            ode_min_step_size=random.random() * 0.001,
            ode_iters=random.randint(10, 50),
            ode_precon_iters=random.randint(1, 3),
            ode_sor=random.random(),
            ode_use_dynamic_moi_rescaling=random.choice([True, False]),
            ode_friction_model=random.choice(
                ['pyramid_model', 'box_model', 'cone_model']),
            ode_cfm=random.random(),
            ode_erp=random.random(),
            ode_contact_max_correcting_vel=random.random(),
            ode_contact_surface_layer=random.random()
        )

        sdf = generate_sdf(test_case, params)
        self.assertIsNotNone(
            sdf,
            'SDF was not generated {} could not be parsed'.format(test_case))
        self.assertEqual(
            sdf.xml_element_name, 'physics',
            'SDF element for file {} should be physics')

        # Test general physics properties
        print(sdf)
        self.assertEqual(sdf.name, params['label'])
        self.assertEqual(sdf.default, '0')
        self.assertEqual(sdf.type, 'ode')
        self.assertTrue(
            np.isclose(
                sdf.max_step_size.value,
                params['max_step_size']))
        self.assertEqual(
            sdf.real_time_factor.value,
            params['real_time_factor'])
        self.assertEqual(
            sdf.real_time_update_rate.value,
            params['real_time_update_rate'])
        self.assertEqual(sdf.max_contacts.value, params['max_contacts'])

        # Test if the other physics engines were not created
        self.assertIsNone(sdf.bullet)
        self.assertIsNone(sdf.simbody)

        # Test ODE default parameters
        print('solver=', sdf.ode.solver._mode, sdf.ode.solver.children.keys())
        self.assertEqual(sdf.ode.solver.type.value, params['ode_solver_type'])
        self.assertTrue(
            np.isclose(
                sdf.ode.solver.min_step_size.value,
                params['ode_min_step_size']))
        self.assertEqual(sdf.ode.solver.iters.value, params['ode_iters'])
        self.assertEqual(
            sdf.ode.solver.precon_iters.value,
            params['ode_precon_iters'])
        self.assertTrue(
            np.isclose(
                sdf.ode.solver.sor.value,
                params['ode_sor']))
        self.assertEqual(
            sdf.ode.solver.use_dynamic_moi_rescaling.value,
            params['ode_use_dynamic_moi_rescaling'])
        self.assertEqual(
            sdf.ode.solver.friction_model.value,
            params['ode_friction_model'])

        self.assertTrue(
            np.isclose(
                sdf.ode.constraints.cfm.value,
                params['ode_cfm']))
        self.assertTrue(
            np.isclose(
                sdf.ode.constraints.erp.value,
                params['ode_erp']))
        self.assertTrue(
            np.isclose(
                sdf.ode.constraints.contact_max_correcting_vel.value,
                params['ode_contact_max_correcting_vel']))
        self.assertTrue(
            np.isclose(
                sdf.ode.constraints.contact_surface_layer.value,
                params['ode_contact_surface_layer']))

    def test_jinja_default_physics(self):
        test_case = 'physics_default'
        sdf = generate_sdf(test_case, None)
        self.assertIsNotNone(
            sdf,
            'SDF was not generated {} could not be parsed'.format(test_case))
        self.assertEqual(
            sdf.xml_element_name, 'physics',
            'SDF element for file {} should be physics')
        print(sdf)
        # Test general physics properties
        self.assertEqual(sdf.name, 'default')
        self.assertEqual(sdf.default, '0')
        self.assertEqual(sdf.type, 'ode')
        self.assertEqual(sdf.max_step_size.value, 0.001)
        self.assertEqual(sdf.real_time_factor.value, 1)
        self.assertEqual(sdf.real_time_update_rate.value, 1000)
        self.assertEqual(sdf.max_contacts.value, 20)

        # Test if the other physics engines were not created
        self.assertIsNone(sdf.bullet)
        self.assertIsNone(sdf.simbody)

        # Test ODE default parameters
        self.assertEqual(sdf.ode.solver.type.value, 'quick')
        self.assertEqual(sdf.ode.solver.min_step_size.value, 0.0001)
        self.assertEqual(sdf.ode.solver.iters.value, 50)
        self.assertEqual(sdf.ode.solver.precon_iters.value, 0)
        self.assertEqual(sdf.ode.solver.sor.value, 1.3)
        self.assertEqual(sdf.ode.solver.use_dynamic_moi_rescaling.value, 0)
        self.assertEqual(sdf.ode.solver.friction_model.value, 'pyramid_model')

        self.assertEqual(sdf.ode.constraints.cfm.value, 0.0)
        self.assertEqual(sdf.ode.constraints.erp.value, 0.2)
        self.assertEqual(
            sdf.ode.constraints.contact_max_correcting_vel.value, 100)
        self.assertEqual(
            sdf.ode.constraints.contact_surface_layer.value, 0.001)

    def test_jinja_inertia_templates(self):

        INPUT_PARAMS = dict(
            inertia_solid_sphere=dict(mass=10, radius=2),
            hollow_sphere_inertia=dict(mass=3, radius=2),
            ellipsoid_inertia=dict(
                mass=10, axis_length_x=2, axis_length_y=3, axis_length_z=4),
            cuboid_inertia=dict(mass=12, length_x=2, length_y=4, length_z=6),
            solid_cylinder_inertia_axis_x=dict(mass=12, radius=10, length=2),
            solid_cylinder_inertia_axis_y=dict(mass=12, radius=10, length=2),
            solid_cylinder_inertia_axis_z=dict(mass=12, radius=10, length=2)
        )

        OUTPUT_PARAMS = dict(
            inertia_solid_sphere=dict(ixx=16, ixy=0, ixz=0, iyy=16, izz=16),
            hollow_sphere_inertia=dict(ixx=8, ixy=0, ixz=0, iyy=8, izz=8),
            ellipsoid_inertia=dict(ixx=50, ixy=0, ixz=0, iyy=40, izz=26),
            cuboid_inertia=dict(ixx=52, ixy=0, ixz=0, iyy=40, izz=20),
            solid_cylinder_inertia_axis_x=dict(
                ixx=600, ixy=0, ixz=0, iyy=304, izz=304),
            solid_cylinder_inertia_axis_y=dict(
                ixx=304, ixy=0, ixz=0, iyy=600, izz=304),
            solid_cylinder_inertia_axis_z=dict(
                ixx=304, ixy=0, ixz=0, iyy=304, izz=600)
        )

        for test_case in INPUT_PARAMS:
            sdf = generate_sdf(test_case, INPUT_PARAMS[test_case])

            self.assertIsNotNone(
                sdf,
                'SDF was not generated {} could not be '
                'parsed'.format(test_case))
            self.assertEqual(
                sdf.xml_element_name, 'inertia',
                'SDF element for file {} should be inertia')

            for param_name in OUTPUT_PARAMS[test_case]:
                self.assertEqual(
                    getattr(sdf, param_name).value,
                    OUTPUT_PARAMS[test_case][param_name],
                    'Parameter {} for inertia {} is incorrect, '
                    'returned={}, expected={}'.format(
                        param_name,
                        test_case,
                        getattr(sdf, param_name).value,
                        OUTPUT_PARAMS[test_case][param_name]
                    ))

    def test_kobuki_robot_description(self):
        template_dir = os.path.abspath(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                '..',
                'examples',
                'robot_description',
                'kobuki',
                'sdf'))

        template = os.path.join(template_dir, 'kobuki.sdf.jinja')
        xml = process_jinja_template(template)
        sdf = parse_sdf(xml)

        self.assertIsNotNone(sdf)
        self.assertEqual(sdf.xml_element_name, 'sdf')
        self.assertIsNotNone(sdf.models)
        self.assertEqual(len(sdf.models), 1)
        self.assertEqual(sdf.models[0].name, 'kobuki')


if __name__ == '__main__':
    unittest.main()
