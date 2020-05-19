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
import numpy as np
import os
from pcg_gazebo.simulation import add_custom_gazebo_resource_path
from pcg_gazebo.collection_managers import EngineManager
from pcg_gazebo.generators.engines import FixedPoseEngine, RandomPoseEngine
from pcg_gazebo.generators.constraints import TangentConstraint, \
    WorkspaceConstraint


FIXED_ENGINE = dict(
    tag='add_fixed_models',
    engine_name='fixed_pose',
    models=['test_static_model'],
    poses=[
        [0, 0, 0, 0, 0, 0]
    ]
)

RANDOM_ENGINE = dict(
    tag='add_random_objects',
    engine_name='random_pose',
    models=[
        'test_box'
    ],
    model_picker='random',
    max_area=0.9,
    no_collision=True,
    max_num=dict(
        test_box=2,
    ),
    policies=[
        dict(
            models=[
                'test_box'
            ],
            config=[
                dict(
                    dofs=['x', 'y'],
                    tag='workspace',
                    workspace='cool_workspace'
                )
            ]
        )
    ],
    constraints=[
        dict(
            model='test_box',
            constraint='tangent_to_ground_plane'
        )
    ]
)

WORKSPACE_CONSTRAINT = dict(
    name='cool_workspace',
    type='workspace',
    frame='world',
    geometry_type='area',
    points=[
        [-6, -4, 0],
        [-3, -4, 0],
        [-3, 0, 0],
        [-6, 0, 0]
    ],
    holes=[
        dict(
            type='circle',
            center=[-5, 0, 0],
            radius=0.2
        )
    ]
)

TANGENT_CONSTRAINT = dict(
    name='tangent_to_ground_plane',
    type='tangent',
    frame='world',
    reference=dict(
        type='plane',
        args=dict(
            origin=[0, 0, 0],
            normal=[0, 0, 1]
        )
    )
)


class TestEnginesConstraintsManagers(unittest.TestCase):
    def test_add_engines(self):
        add_custom_gazebo_resource_path(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'gazebo_models'))
        manager = EngineManager()

        manager.add(**FIXED_ENGINE)
        self.assertIn('add_fixed_models', manager.tags)
        self.assertEqual(manager.size, 1)

        engine = manager.get('add_fixed_models')
        self.assertIsNotNone(engine)
        self.assertIsInstance(engine, FixedPoseEngine)

        manager.add(**RANDOM_ENGINE)
        self.assertIn('add_random_objects', manager.tags)
        self.assertEqual(manager.size, 2)

        engine = manager.get('add_random_objects')
        self.assertIsNotNone(engine)
        self.assertIsInstance(engine, RandomPoseEngine)

    def test_add_constraint(self):
        add_custom_gazebo_resource_path(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'gazebo_models'))
        manager = EngineManager()
        manager.add_constraint(**WORKSPACE_CONSTRAINT)
        self.assertTrue(manager.has_constraint('cool_workspace'))
        const = manager.get_constraint('cool_workspace')
        self.assertIsInstance(const, WorkspaceConstraint)

        manager.add_constraint(**TANGENT_CONSTRAINT)
        self.assertTrue(manager.has_constraint('tangent_to_ground_plane'))
        const = manager.get_constraint('tangent_to_ground_plane')
        self.assertIsInstance(const, TangentConstraint)

    def test_run_random_engine(self):
        add_custom_gazebo_resource_path(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'gazebo_models'))
        manager = EngineManager()
        manager.add_constraint(**WORKSPACE_CONSTRAINT)
        self.assertTrue(manager.has_constraint('cool_workspace'))
        const = manager.get_constraint('cool_workspace')
        self.assertIsInstance(const, WorkspaceConstraint)

        manager.add_constraint(**TANGENT_CONSTRAINT)
        self.assertTrue(manager.has_constraint('tangent_to_ground_plane'))
        const = manager.get_constraint('tangent_to_ground_plane')
        self.assertIsInstance(const, TangentConstraint)

        manager.add(**RANDOM_ENGINE)
        self.assertIn('add_random_objects', manager.tags)
        self.assertEqual(manager.size, 1)

        engine = manager.get('add_random_objects')
        self.assertIsNotNone(engine)
        self.assertIsInstance(engine, RandomPoseEngine)

        models = engine.run()
        self.assertIsNotNone(models)
        self.assertIsInstance(models, list)
        self.assertEqual(len(models), 2)

    def test_run_fixed_pose_engine(self):
        add_custom_gazebo_resource_path(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'gazebo_models'))
        manager = EngineManager()

        manager.add(**FIXED_ENGINE)
        self.assertIn('add_fixed_models', manager.tags)
        self.assertEqual(manager.size, 1)

        engine = manager.get('add_fixed_models')
        self.assertIsNotNone(engine)
        self.assertIsInstance(engine, FixedPoseEngine)

        models = engine.run()
        self.assertIsNotNone(models)
        self.assertIsInstance(models, list)
        self.assertEqual(len(models), 1)

        self.assertEqual(np.sum(models[0].pose.position), 0)
        self.assertEqual(np.sum(models[0].pose.quat[0:3]), 0)


if __name__ == '__main__':
    unittest.main()
