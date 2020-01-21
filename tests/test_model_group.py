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
import unittest
import numpy
import os
import shutil
import getpass
from pcg_gazebo.utils import generate_random_string
from pcg_gazebo.simulation import ModelGroup, Light, SimulationModel
from pcg_gazebo.simulation.properties import Pose
from pcg_gazebo.generators.creators import box_factory
from pcg_gazebo.parsers import parse_sdf, parse_sdf_config
from pcg_gazebo.parsers.sdf import create_sdf_element


class TestModelGroup(unittest.TestCase):
    def test_group_name(self):
        valid_name = generate_random_string(size=3)
        group = ModelGroup(name=valid_name)
        self.assertEqual(valid_name, group.name)

        invalid_names = [
            10,
            None,
            dict(),
            list(),
            ''
        ]

        for name in invalid_names:
            with self.assertRaises(AssertionError):
                group = ModelGroup(name=name)

    def test_group_init_pose(self):
        # Set initial pose with Pose object
        pose = Pose.random()
        group = ModelGroup(pose=pose)

        self.assertEqual(numpy.sum(group.pose.position - pose.position), 0)
        self.assertEqual(numpy.sum(group.pose.quat - pose.quat), 0)

        # Test invalid pose objects
        invalid_poses = [
            dict(),
            list(),
            'asd',
            '',
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            None
        ]

        for pose in invalid_poses:
            with self.assertRaises(AssertionError):
                group.pose = pose

    def test_add_models(self):
        group = ModelGroup()
        boxes = box_factory(
            size="__import__('numpy').random.random((2, 3))",
            mass="__import__('numpy').random.random(2)",
            use_permutation=True,
            name='box'
        )

        self.assertEqual(len(boxes), 4)

        for box in boxes:
            group.add_model(box.name, box)
            self.assertIn(box.name, group.models)
            self.assertTrue(group.model_exists(box.name))

        self.assertEqual(group.n_models, 4)

        meshes = group.get_meshes()
        self.assertEqual(len(meshes), 4)

        group.reset_models()
        self.assertEqual(group.n_models, 0)

        for box in boxes:
            group.add_model(box.name, box)
        self.assertEqual(group.n_models, 4)

        for box in boxes:
            self.assertTrue(group.rm_model(box.name))
        self.assertEqual(group.n_models, 0)

    def test_bounds(self):
        box_1 = box_factory(
            size=[
                [1, 1, 1]
            ],
            mass=1,
            use_permutation=True,
            name='box_1'
        )[0]

        self.assertTrue(box_1.name, 'box_1')

        box_2 = box_factory(
            size=[
                [1, 1, 1]
            ],
            mass=1,
            use_permutation=True,
            name='box_2'
        )[0]

        self.assertTrue(box_2.name, 'box_2')

        # Test bounds for different positions of the boxes
        box_1.pose = Pose.random_position()
        box_2.pose = Pose.random_position()

        group = ModelGroup()
        group.add_model(box_1.name, box_1)
        group.add_model(box_2.name, box_2)

        self.assertEqual(group.n_models, 2)

        bounds = group.get_bounds()

        ref_bounds = numpy.array(
            [
                [numpy.min([box_1.pose.x, box_2.pose.x]) - 1,
                 numpy.min([box_1.pose.y, box_2.pose.y]) - 1,
                 numpy.min([box_1.pose.z, box_2.pose.z]) - 1],
                [numpy.max([box_1.pose.x, box_2.pose.x]) + 1,
                 numpy.max([box_1.pose.y, box_2.pose.y]) + 1,
                 numpy.max([box_1.pose.z, box_2.pose.z]) + 1]
            ])

        self.assertAlmostEqual(numpy.sum(ref_bounds - bounds), 0)

    def test_retrieve_model(self):
        box = box_factory(
            size=[
                [1, 1, 1]
            ],
            mass=1,
            use_permutation=True,
            name='box'
        )[0]

        group = ModelGroup(name='group')
        group.pose = Pose.random()

        box.pose = Pose.random()
        group.add_model('box', box)

        model = group.get_model('box')

        self.assertTrue(box.pose + group.pose, model.pose)

        models = group.get_models()
        self.assertEqual(len(models), 1)

        group_model = group.get_model('box')

        self.assertIsNotNone(group_model)

    def test_add_light(self):
        sdf = """
        <light type="point" name="point_light">
            <pose>0 2 2 0 0 0</pose>
            <diffuse>1 0 0 1</diffuse>
            <specular>.1 .1 .1 1</specular>
            <attenuation>
                <range>20</range>
                <linear>0.2</linear>
                <constant>0.8</constant>
                <quadratic>0.01</quadratic>
            </attenuation>
            <cast_shadows>false</cast_shadows>
        </light>
        """

        light = Light.from_sdf(parse_sdf(sdf))
        group = ModelGroup()

        group.add_light('light', light)
        self.assertEqual(group.n_lights, 1)
        self.assertIsNotNone(group.get_light('light'))

        group_light = group.get_light('light')

        self.assertIsNotNone(group_light)

    def test_model_group_to_sdf(self):
        sdf_light = """
        <light type="point" name="point_light">
            <pose>0 2 2 0 0 0</pose>
            <diffuse>1 0 0 1</diffuse>
            <specular>.1 .1 .1 1</specular>
            <attenuation>
                <range>20</range>
                <linear>0.2</linear>
                <constant>0.8</constant>
                <quadratic>0.01</quadratic>
            </attenuation>
            <cast_shadows>false</cast_shadows>
        </light>
        """
        light = Light.from_sdf(parse_sdf(sdf_light))
        box = box_factory(
            size=[
                [1, 1, 1]
            ],
            mass=1,
            use_permutation=True,
            name='box'
        )[0]

        group = ModelGroup(name='test')
        group.add_light('light', light)
        group.add_model('box', box)

        sdf_models, sdf_lights, sdf_includes = group.to_sdf(use_include=False)

        self.assertEqual(len(sdf_models), 1)
        self.assertEqual(len(sdf_lights), 1)
        self.assertEqual(len(sdf_includes), 0)

        group_box = group.get_model('box', with_group_prefix=False)
        self.assertIsNotNone(group_box)
        self.assertEqual(group_box.name, 'box')

        group_box = group.get_model('box', with_group_prefix=True)
        self.assertIsNotNone(group_box)
        self.assertEqual(group_box.name, 'test/box')

        group_models = group.get_models(with_group_prefix=False)
        self.assertIsInstance(group_models, dict)
        self.assertEqual(len(group_models), 1)
        self.assertIn('box', group_models)

        group_models = group.get_models(with_group_prefix=True)
        self.assertIsInstance(group_models, dict)
        self.assertEqual(len(group_models), 1)
        self.assertIn('test/box', group_models)

        group_light = group.get_light('light', with_group_prefix=False)
        self.assertIsNotNone(group_light)
        self.assertEqual(group_light.name, 'light')

        group_light = group.get_light('light', with_group_prefix=True)
        self.assertIsNotNone(group_light)
        self.assertEqual(group_light.name, 'test/light')

        group_lights = group.get_lights(with_group_prefix=False)
        self.assertIsInstance(group_lights, dict)
        self.assertEqual(len(group_lights), 1)
        self.assertIn('light', group_lights)

        group_lights = group.get_lights(with_group_prefix=True)
        self.assertIsInstance(group_lights, dict)
        self.assertEqual(len(group_lights), 1)
        self.assertIn('test/light', group_lights)

    def test_nested_model_groups(self):
        sdf_light = """
        <light type="point" name="point_light">
            <pose>0 2 2 0 0 0</pose>
            <diffuse>1 0 0 1</diffuse>
            <specular>.1 .1 .1 1</specular>
            <attenuation>
                <range>20</range>
                <linear>0.2</linear>
                <constant>0.8</constant>
                <quadratic>0.01</quadratic>
            </attenuation>
            <cast_shadows>false</cast_shadows>
        </light>
        """
        light = Light.from_sdf(parse_sdf(sdf_light))

        box_1 = box_factory(
            size=[
                [1, 1, 1]
            ],
            mass=1,
            use_permutation=True,
            name='box'
        )[0]

        box_2 = box_factory(
            size=[
                [2, 2, 2]
            ],
            mass=2,
            use_permutation=True,
            name='box'
        )[0]

        nested_group = ModelGroup(name='nested')
        root_group = ModelGroup(name='root')

        nested_group.add_light('light', light)
        nested_group.add_model('box', box_1)

        root_group.add_model('box', box_2)
        root_group.add_model('nested', nested_group)

        # Trying to get light from the root group, should return None
        gl = root_group.get_light('light')
        self.assertIsNone(gl)
        gl = root_group.get_light('nested/light')
        self.assertIsNotNone(gl)

        # Trying to get boxes from both groups
        gb = root_group.get_model('box')
        self.assertIsNotNone(gb)
        self.assertEqual(gb.links[gb.link_names[0]].inertial.mass, 2)

        gb = root_group.get_model('nested/box')
        self.assertIsNotNone(gb)
        self.assertEqual(gb.links[gb.link_names[0]].inertial.mass, 1)

        # Request all models from nested group (with and without prefix)
        group_models = nested_group.get_models(with_group_prefix=False)
        self.assertIsInstance(group_models, dict)
        self.assertEqual(len(group_models), 1)
        self.assertIn('box', group_models)

        group_models = nested_group.get_models(with_group_prefix=True)
        self.assertIsInstance(group_models, dict)
        self.assertEqual(len(group_models), 1)
        self.assertIn('nested/box', group_models)

        # Request all lights from nested group (with and without prefix)
        group_lights = nested_group.get_lights(with_group_prefix=False)
        self.assertIsInstance(group_lights, dict)
        self.assertEqual(len(group_lights), 1)
        self.assertIn('light', group_lights)

        group_lights = nested_group.get_lights(with_group_prefix=True)
        self.assertIsInstance(group_lights, dict)
        self.assertEqual(len(group_lights), 1)
        self.assertIn('nested/light', group_lights)

        # Request all models from root group (with and without prefix)
        group_models = root_group.get_models(with_group_prefix=False)
        self.assertIsInstance(group_models, dict)
        self.assertEqual(len(group_models), 2)
        self.assertIn('box', group_models)
        self.assertIn('nested/box', group_models)

        group_models = root_group.get_models(with_group_prefix=True)
        self.assertIsInstance(group_models, dict)
        self.assertEqual(len(group_models), 2)
        self.assertIn('root/box', group_models)
        self.assertIn('root/nested/box', group_models)

        # Request all lights from root group (with and without prefix)
        group_lights = root_group.get_lights(with_group_prefix=False)
        self.assertIsInstance(group_lights, dict)
        self.assertEqual(len(group_lights), 1)
        self.assertIn('nested/light', group_lights)

        group_lights = root_group.get_lights(with_group_prefix=True)
        self.assertIsInstance(group_lights, dict)
        self.assertEqual(len(group_lights), 1)
        self.assertIn('root/nested/light', group_lights)

    def test_load_from_sdf(self):
        lights = [Light(name=generate_random_string(5)) for _ in range(3)]
        models = [SimulationModel(name=generate_random_string(5))
                  for _ in range(2)]

        # Test import from a list of SDF elements
        sdf_elements = [light.to_sdf() for light in lights] + \
            [model.to_sdf() for model in models]
        group = ModelGroup.from_sdf(sdf_elements)
        self.assertIsNotNone(group)
        self.assertEqual(group.n_models, len(models))
        self.assertEqual(group.n_lights, len(lights))

        # Test import from one single SDF element
        sdf = create_sdf_element('sdf')
        for model in models:
            sdf.add_model(model.name, model.to_sdf())
        for light in lights:
            sdf.add_light(light.name, light.to_sdf())

        group = ModelGroup.from_sdf(sdf)
        self.assertIsNotNone(group)
        self.assertEqual(group.n_models, len(models))
        self.assertEqual(group.n_lights, len(lights))

    def test_convert_wrong_model_sdf(self):
        lights = [Light(name=generate_random_string(5)) for _ in range(3)]
        models = [SimulationModel(name=generate_random_string(5))
                  for _ in range(2)]

        sdf_elements = [light.to_sdf() for light in lights] + \
            [model.to_sdf() for model in models]

        group = ModelGroup.from_sdf(sdf_elements)
        self.assertIsNotNone(group)
        self.assertEqual(group.n_models, len(models))
        self.assertEqual(group.n_lights, len(lights))

        self.assertIsNone(group.to_sdf(type='model'))

    def test_convert_lights_models_group_to_sdfs(self):
        lights = [Light(name=generate_random_string(5)) for _ in range(3)]
        light_names = [obj.name for obj in lights]
        models = [SimulationModel(name=generate_random_string(5))
                  for _ in range(2)]
        model_names = [obj.name for obj in models]

        sdf_elements = [light.to_sdf() for light in lights] + \
            [model.to_sdf() for model in models]

        group = ModelGroup.from_sdf(sdf_elements)
        group.name = generate_random_string(5)
        self.assertIsNotNone(group)
        self.assertEqual(group.n_models, len(models))
        self.assertEqual(group.n_lights, len(lights))

        # Convert to separate SDF elements
        sdf_models, sdf_lights, sdf_includes = group.to_sdf()
        self.assertEqual(len(sdf_models), len(models))
        for tag in sdf_models:
            self.assertIn(sdf_models[tag].name.replace(
                '{}/'.format(group.name), ''), model_names)
        self.assertEqual(len(sdf_lights), len(lights))
        for tag in sdf_lights:
            self.assertIn(sdf_lights[tag].name.replace(
                '{}/'.format(group.name), ''), light_names)
        self.assertEqual(len(sdf_includes), 0)

        # Convert to world
        sdf_world = group.to_sdf(type='world')
        self.assertIsNotNone(sdf_world)
        self.assertEqual(sdf_world.xml_element_name, 'world')
        self.assertIsNotNone(sdf_world.models)
        self.assertEqual(len(sdf_world.models), 1)
        self.assertEqual(len(sdf_world.models[0].models), len(models))
        for i in range(len(sdf_world.models[0].models)):
            self.assertIn(sdf_world.models[0].models[i].name, model_names)
        self.assertEqual(len(sdf_world.lights), len(lights))
        for i in range(len(sdf_world.lights)):
            self.assertIn(sdf_world.lights[i].name, light_names)

    def test_convert_models_group_to_sdf(self):
        models = [SimulationModel(name=generate_random_string(5))
                  for _ in range(2)]
        model_names = [obj.name for obj in models]
        sdf_elements = [model.to_sdf() for model in models]

        group = ModelGroup.from_sdf(sdf_elements)
        self.assertIsNotNone(group)
        self.assertEqual(group.n_models, len(models))
        self.assertEqual(group.n_lights, 0)

        # Convert to separate SDF elements
        sdf_models, sdf_lights, sdf_includes = group.to_sdf()
        self.assertEqual(len(sdf_models), len(models))
        for tag in sdf_models:
            self.assertIn(sdf_models[tag].name.replace(
                '{}/'.format(group.name), ''), model_names)
        self.assertEqual(len(sdf_lights), 0)
        self.assertEqual(len(sdf_includes), 0)

        # Convert to world
        sdf_world = group.to_sdf(type='world')
        self.assertIsNotNone(sdf_world)
        self.assertEqual(sdf_world.xml_element_name, 'world')
        self.assertIsNotNone(sdf_world.models)
        self.assertEqual(len(sdf_world.models), 1)
        self.assertEqual(len(sdf_world.models[0].models), len(models))
        for i in range(len(sdf_world.models[0].models)):
            self.assertIn(sdf_world.models[0].models[i].name, model_names)

        # Convert to SDF with model group
        sdf = group.to_sdf(type='sdf')
        self.assertIsNotNone(sdf)
        self.assertEqual(sdf.xml_element_name, 'sdf')
        self.assertIsNotNone(sdf_world.models)
        self.assertEqual(len(sdf_world.models), 1)
        self.assertEqual(len(sdf_world.models[0].models), len(models))
        for i in range(len(sdf_world.models[0].models)):
            self.assertIn(sdf_world.models[0].models[i].name, model_names)

        # Convert to model with nested models
        sdf_model = group.to_sdf(type='model')
        self.assertIsNotNone(sdf_model)
        self.assertIsNotNone(sdf_model.models)
        self.assertEqual(len(sdf_model.models), len(models))
        for i in range(len(sdf_model.models)):
            self.assertIn(sdf_model.models[i].name, model_names)

    def test_convert_lights_group_to_sdf(self):
        lights = [Light(name=generate_random_string(5)) for _ in range(3)]
        light_names = [obj.name for obj in lights]
        sdf_elements = [light.to_sdf() for light in lights]

        group = ModelGroup.from_sdf(sdf_elements)
        self.assertIsNotNone(group)
        self.assertEqual(group.n_models, 0)
        self.assertEqual(group.n_lights, len(lights))

        # Convert to separate SDF elements
        sdf_models, sdf_lights, sdf_includes = group.to_sdf()
        self.assertEqual(len(sdf_models), 0)
        self.assertEqual(len(sdf_lights), len(lights))
        for tag in sdf_lights:
            self.assertIn(sdf_lights[tag].name.replace(
                '{}/'.format(group.name), ''), light_names)
        self.assertEqual(len(sdf_includes), 0)

        # Convert to world
        sdf_world = group.to_sdf(type='world')
        self.assertIsNotNone(sdf_world)
        self.assertEqual(sdf_world.xml_element_name, 'world')
        self.assertIsNone(sdf_world.models)
        self.assertIsNotNone(sdf_world.lights)
        self.assertEqual(len(sdf_world.lights), len(lights))
        for i in range(len(sdf_world.lights)):
            self.assertIn(sdf_world.lights[i].name, light_names)

        # Convert to SDF with light elements
        sdf = group.to_sdf(type='sdf')
        self.assertIsNotNone(sdf)
        self.assertEqual(sdf.xml_element_name, 'sdf')
        self.assertIsNotNone(sdf_world.lights)
        self.assertEqual(len(sdf_world.lights), len(lights))
        for i in range(len(sdf_world.lights)):
            self.assertIn(sdf_world.lights[i].name, light_names)

    def test_export_models_to_gazebo_model(self):
        models = [SimulationModel(name=generate_random_string(5))
                  for _ in range(2)]
        model_names = [obj.name for obj in models]
        sdf_elements = [model.to_sdf() for model in models]

        group = ModelGroup.from_sdf(sdf_elements)
        group.name = generate_random_string(5)
        group.pose = Pose.random()
        self.assertIsNotNone(group)
        self.assertEqual(group.n_models, len(models))
        self.assertEqual(group.n_lights, 0)

        # Convert to separate SDF elements
        sdf_models, sdf_lights, sdf_includes = group.to_sdf()
        self.assertEqual(len(sdf_models), len(models))
        for tag in sdf_models:
            self.assertIn(sdf_models[tag].name.replace(
                '{}/'.format(group.name), ''), model_names)
        self.assertEqual(len(sdf_lights), 0)
        self.assertEqual(len(sdf_includes), 0)

        # Convert to default Gazebo model
        self.assertIsNotNone(group.to_gazebo_model())
        default_dir = os.path.join(
            os.path.expanduser('~'), '.gazebo', 'models')
        model_dir = os.path.join(default_dir, group.name)

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

        self.assertEqual(sdf_config.name.value, group.name)
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

        # Parse the SDF file
        sdf = parse_sdf(os.path.join(model_dir, 'model.sdf'))
        self.assertIsNotNone(sdf)
        self.assertEqual(sdf.xml_element_name, 'sdf')
        self.assertIsNotNone(sdf.models)
        self.assertEqual(len(sdf.models), 1)
        for i, j in zip(sdf.models[0].pose.value, group.pose.to_sdf().value):
            self.assertTrue(numpy.isclose(i, j))
        self.assertEqual(len(sdf.models[0].models), len(models))

        for i in range(len(sdf.models[0].models)):
            self.assertEqual(
                sdf.models[0].models[i].pose.value, [
                    0 for _ in range(6)])
            self.assertIn(sdf.models[0].models[i].name, model_names)

        # Delete generated Gazebo model directory
        shutil.rmtree(model_dir)

        # Rename group
        group.name = generate_random_string(5)

        # Export group with individually exported models
        self.assertIsNotNone(group.to_gazebo_model(nested=False))

        # Check if model folders exist
        self.assertTrue(os.path.isdir(os.path.join(default_dir, group.name)))
        for name in model_names:
            self.assertTrue(os.path.isdir(os.path.join(default_dir, name)))

        # Parse the SDF file for the main model
        sdf = parse_sdf(os.path.join(default_dir, group.name, 'model.sdf'))
        self.assertIsNotNone(sdf)
        self.assertEqual(sdf.xml_element_name, 'sdf')
        self.assertIsNotNone(sdf.models)
        self.assertEqual(len(sdf.models), 1)

        self.assertIsNone(sdf.models[0].models)
        self.assertIsNotNone(sdf.models[0].includes)
        self.assertEqual(len(sdf.models[0].includes), len(models))

        for i, j in zip(sdf.models[0].pose.value, group.pose.to_sdf().value):
            self.assertTrue(numpy.isclose(i, j))

        for i in range(len(sdf.models[0].includes)):
            self.assertEqual(
                sdf.models[0].includes[i].pose.value, [
                    0 for _ in range(6)])
            self.assertIn(sdf.models[0].includes[i].name.value, model_names)
            self.assertEqual(
                sdf.models[0].includes[i].uri.value,
                'model://{}'.format(sdf.models[0].includes[i].name.value))

        shutil.rmtree(os.path.join(default_dir, group.name))
        for name in model_names:
            shutil.rmtree(os.path.join(default_dir, name))


if __name__ == '__main__':
    unittest.main()
