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
from pcg_gazebo.generators import ModelGroupGenerator
from pcg_gazebo.generators.creators import box_factory


FIXED_ENGINE = dict(
    tag='add_fixed_models',
    engine_name='fixed_pose',
    models=['box_floor'],
    poses=[
        [0, 0, 0, 0, 0, 0]
    ]
)

RANDOM_ENGINE = dict(
    tag='add_random_objects',
    engine_name='random_pose',
    models=[
        'box'
    ],
    model_picker='size',
    max_area=0.9,
    no_collision=True,
    max_num=dict(
        box=2,
    ),
    policies=[
        dict(
            models=[
                'box'
            ],
            config=[
                dict(
                    dofs=['x', 'y'],
                    policy=dict(
                        name='workspace',
                        args='cool_workspace'
                    )
                )
            ]
        )
    ],
    constraints=[
        dict(
            model='box',
            constraint='tangent_to_ground_plane'
        )
    ]
)

WORKSPACE_CONSTRAINT = dict(
    name='cool_workspace',
    type='workspace',
    frame='world',
    geometry=dict(
        type='area',
        description=dict(
          points=[
              [-6, -4, 0],
              [-3, -4, 0],
              [-3, 0, 0],
              [-6, 0, 0]
          ]
        )
    ),
    holes=[
        dict(
            type='circle',
            description=dict(
                center=[-5, 0, 0],
                radius=0.2
            )
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

BOX_MODEL = box_factory(
    size=[
        [1, 1, 1]
    ],
    mass=1,
    use_permutation=True,
    name='box'
)[0]
BOX_MODEL.name = 'box'

BOX_FLOOR_MODEL = box_factory(
    size=[
        [1, 1, 0.01]
    ],
    mass=1,
    use_permutation=True,
    name='box_floor'
)[0]
BOX_FLOOR_MODEL.name = 'box_floor'


class TestModelGroupGenerator(unittest.TestCase):
    def test_add_engines_and_constraints_as_obj(self):
        # Add individually
        generator = ModelGroupGenerator()
        self.assertTrue(generator.add_constraint(**TANGENT_CONSTRAINT))
        self.assertTrue(generator.add_constraint(**WORKSPACE_CONSTRAINT))
        self.assertTrue(generator.add_engine(**FIXED_ENGINE))
        self.assertTrue(generator.add_engine(**RANDOM_ENGINE))
        self.assertTrue(generator.add_asset(BOX_MODEL))
        self.assertTrue(generator.add_asset(BOX_FLOOR_MODEL))

        self.assertTrue(generator.is_asset('box_floor'))
        self.assertTrue(generator.is_asset('box'))

        generator.set_asset_as_ground_plane('box_floor')

        # Run all engines and retrieve model group
        group = generator.run('test')
        self.assertIsNotNone(group)
        self.assertEqual(group.n_models, 3)

    def test_add_engines_and_constraints_as_dict(self):
        # Add as configuration
        config = dict(
            assets=dict(
                ground_plane=['box_floor'],
                assets=[
                    dict(description=BOX_FLOOR_MODEL),
                    dict(description=BOX_MODEL),
                ]
            ),
            engines=[
                FIXED_ENGINE,
                RANDOM_ENGINE
            ],
            constraints=[
                WORKSPACE_CONSTRAINT,
                TANGENT_CONSTRAINT
            ]
        )

        generator = ModelGroupGenerator.from_dict(config)

        self.assertTrue(generator.is_asset('box_floor'))
        self.assertTrue(generator.is_asset('box'))

        group = generator.run('test')
        self.assertIsNotNone(group)
        self.assertEqual(group.n_models, 3)


if __name__ == '__main__':
    unittest.main()
