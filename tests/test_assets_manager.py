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
import os
from pcg_gazebo.collection_managers import AssetsManager
from pcg_gazebo.generators.creators import config2models
from pcg_gazebo.utils import generate_random_string
from pcg_gazebo.simulation import ModelGroup, SimulationModel

CUR_DIR = os.path.dirname(os.path.abspath(__file__))


class TestAssetManager(unittest.TestCase):
    def test_default_models(self):
        man = AssetsManager.get_instance()
        self.assertIn('sun', man.tags)
        self.assertIn('ground_plane', man.tags)

    def test_add_assets_from_dict(self):
        box_factory_config = dict(
            type='box',
            args=dict(
                size=[2, 2, 0.02],
                mass=10,
                name='box',
                pose=[0, 0, 0, 0, 0, 0],
                color='random'
            )
        )

        box_config = dict(
            type='box',
            args=dict(
                size=[2, 2, 2],
                mass=10,
                name='box',
                pose=[0, 0, 1, 0, 0, 0],
                color='random'
            )
        )

        # Setup the dictionary input configuration for
        # the assets manager
        config = dict(
            ground_plane=['box_floor'], assets=[
                dict(
                    tag='box_floor', description=SimulationModel.from_sdf(
                        config2models(box_factory_config)[0])), dict(
                    tag='box', description=box_config)])

        manager = AssetsManager.get_instance()
        manager.from_dict(config)

        self.assertIn('box_floor', manager.tags)
        self.assertIn('box', manager.tags)

        self.assertIn('box_floor', manager.ground_planes)

        model = manager.get('box_floor')
        self.assertIsInstance(model, SimulationModel)
        self.assertTrue(model.is_ground_plane)

        model = manager.get('box')
        self.assertIsInstance(model, SimulationModel)

        manager.reset()
        self.assertGreaterEqual(len(manager.tags), 0)

    def test_add_simulation_models(self):
        box_config = dict(
            type='box',
            args=dict(
                size=[2, 2, 2],
                mass=10,
                name='box',
                pose=[0, 0, 1, 0, 0, 0],
                color='random'
            )
        )

        model = SimulationModel.from_sdf(config2models(box_config)[0])
        name = generate_random_string(5)

        manager = AssetsManager.get_instance()
        manager.add(model, name)
        self.assertIn(name, manager.tags)
        self.assertIsInstance(manager.get(name), SimulationModel)

        manager.reset()
        self.assertGreaterEqual(len(manager.tags), 0)

    def test_add_model_groups(self):
        box_factory_config = dict(
            type='box_factory',
            args=dict(
                size="__import__('numpy').random.random((2, 3))",
                use_permutation=True,
                name='box',
                color='xkcd'
            )
        )

        model_sdfs = config2models(box_factory_config)
        group_name = generate_random_string(5)
        group = ModelGroup(name=group_name)
        for name, sdf in zip(['box_0', 'box_1'], model_sdfs):
            group.add_model(name, SimulationModel.from_sdf(sdf))

        manager = AssetsManager.get_instance()
        manager.add(group, group_name)
        self.assertIn(group_name, manager.tags)

        self.assertIsInstance(manager.get(group_name), ModelGroup)

        manager.reset()
        self.assertGreaterEqual(len(manager.tags), 0)

    def test_get_gazebo_models(self):
        manager = AssetsManager.get_instance()

        manager.add_custom_gazebo_resource_path(
            os.path.join(CUR_DIR, 'gazebo_models'))

        self.assertGreaterEqual(manager.size, 2)

        self.assertIn('test_joint_fixed', manager.tags)
        self.assertIn('test_static_model', manager.tags)

        print(manager.tags)

        model_1 = manager.get('test_joint_fixed')
        self.assertIsNotNone(model_1)

        model_2 = manager.get('test_static_model')
        self.assertIsNotNone(model_2)
        self.assertFalse(model_2.is_ground_plane)

        manager.set_asset_as_ground_plane('test_static_model')
        model_2 = manager.get('test_static_model')
        self.assertIsNotNone(model_2)
        self.assertTrue(model_2.is_ground_plane)

        manager.reset()
        self.assertGreaterEqual(len(manager.tags), 3)

    def test_add_factory_config(self):
        manager = AssetsManager.get_instance()

        added_models = list()
        # Add box factory
        box_config = dict(
            type='box',
            args=dict(
                size=[2, 2, 2],
                mass=10,
                name='box',
                pose=[0, 0, 1, 0, 0, 0],
                color='random'
            )
        )

        added_models.append(generate_random_string(5))
        manager.add(box_config, tag=added_models[-1])
        self.assertIn(added_models[-1], manager.tags)

        model = manager.get(added_models[-1])
        self.assertIsNotNone(model)
        self.assertIn('<box>', model.to_sdf().to_xml_as_str())
        self.assertEqual(len(model.links), 1)

        # Add cylinder factory
        cylinder_config = dict(
            type='cylinder',
            args=dict(
                radius=3,
                length=2,
                mass=10,
                name='cylinder',
                pose=[0, 0, 1, 0, 0, 0]
            )
        )

        added_models.append(generate_random_string(5))
        manager.add(cylinder_config, tag=added_models[-1])
        self.assertIn(added_models[-1], manager.tags)

        model = manager.get(added_models[-1])
        self.assertIsNotNone(model)
        self.assertIn('<cylinder>', model.to_sdf().to_xml_as_str())
        self.assertEqual(len(model.links), 1)

        # Add sphere factory
        sphere_config = dict(
            type='sphere',
            args=dict(
                radius=3,
                name='sphere',
                mass=10,
                pose=[0, 0, 1.5, 0, 0, 0]
            )
        )

        added_models.append(generate_random_string(5))
        manager.add(sphere_config, tag=added_models[-1])
        self.assertIn(added_models[-1], manager.tags)

        model = manager.get(added_models[-1])
        self.assertIsNotNone(model)
        self.assertIn('<sphere>', model.to_sdf().to_xml_as_str())
        self.assertEqual(len(model.links), 1)

        # Add mesh factory
        mesh_config = dict(
            type='mesh',
            args=dict(
                visual_mesh=os.path.join(CUR_DIR, 'meshes', 'monkey.stl'),
                visual_mesh_scale=[1, 1, 1],
                use_approximated_collision=True,
                name='mesh',
                color='xkcd'
            )
        )

        added_models.append(generate_random_string(5))
        manager.add(mesh_config, tag=added_models[-1])
        self.assertIn(added_models[-1], manager.tags)

        model = manager.get(added_models[-1])
        self.assertIsNotNone(model)
        self.assertIn('<mesh>', model.to_sdf().to_xml_as_str())
        self.assertEqual(len(model.links), 1)

        manager.reset()
        self.assertGreaterEqual(len(manager.tags), 0)

    def test_add_duplicated_models(self):
        box_config = dict(
            type='box',
            args=dict(
                size=[2, 2, 2],
                mass=10,
                name='box',
                pose=[0, 0, 1, 0, 0, 0],
                color='random'
            )
        )

        model = SimulationModel.from_sdf(config2models(box_config)[0])
        name = generate_random_string(5)

        manager = AssetsManager.get_instance()
        self.assertTrue(manager.add(model, name))
        self.assertIn(name, manager.tags)
        self.assertIsInstance(manager.get(name), SimulationModel)
        self.assertFalse(manager.add(model, name, overwrite=False))

        manager.reset()
        self.assertGreaterEqual(len(manager.tags), 0)


if __name__ == '__main__':
    unittest.main()
