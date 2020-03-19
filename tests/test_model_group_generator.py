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
                    tag='workspace',
                    workspace='cool_workspace'
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

        generator.set_model_as_ground_plane('box_floor')

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

        generator = ModelGroupGenerator(**config)

        self.assertTrue(generator.is_asset('box_floor'))
        self.assertTrue(generator.is_asset('box'))

        group = generator.run('test')
        self.assertIsNotNone(group)
        self.assertEqual(group.n_models, 3)

    def test_crate_example(self):
        generator = ModelGroupGenerator('full_crate')

        cur_dir = os.path.dirname(os.path.abspath(__file__))
        visual_mesh_filename = os.path.join(
            cur_dir,
            '..',
            'examples',
            'meshes',
            'crate.stl')
        self.assertTrue(os.path.isfile(visual_mesh_filename))

        generator.add_asset(
            tag='crate',
            description=dict(
                type='mesh',
                args=dict(
                    visual_mesh='file://' + visual_mesh_filename,
                    visual_mesh_scale=[1, 1, 1],
                    use_approximated_collision=False,
                    name='crate',
                    color='xkcd'
                )
            )
        )

        random_scalar_inline = \
            "max(0.05, 0.08 * __import__('numpy').random.random())"

        generator.add_asset(
            tag='crate_ball',
            description=dict(
                type='sphere',
                args=dict(
                    radius=random_scalar_inline,
                    name='sphere',
                    mass="max(0.1, __import__('numpy').random.random())",
                    color='xkcd'
                )
            )
        )

        generator.add_asset(
            tag='crate_cuboid',
            description=dict(
                type='box',
                args=dict(
                    size="0.08 * __import__('numpy').random.random(3)",
                    name='cuboid',
                    mass="max(0.01, __import__('numpy').random.random())",
                    color='xkcd'
                )
            )
        )

        generator.add_asset(
            tag='crate_cylinder',
            description=dict(
                type='cylinder',
                args=dict(
                    length=random_scalar_inline,
                    radius=random_scalar_inline,
                    name='cuboid',
                    mass="max(0.01, __import__('numpy').random.random())",
                    color='xkcd'
                )
            )
        )

        generator.add_constraint(
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

        generator.add_constraint(
            name='crate_base',
            type='workspace',
            frame='world',
            geometry=dict(
                type='area',
                description=dict(
                    points=[
                        [-0.5, -0.4, 0],
                        [-0.5, 0.4, 0],
                        [0.5, 0.4, 0],
                        [0.5, -0.4, 0]
                    ]
                )
            )
        )

        generator.add_engine(
            tag='crate_engine',
            engine_name='fixed_pose',
            models=['crate'],
            poses=[
                [0, 0, 0, 0, 0, 0]
            ],
            constraints=[
                dict(
                    model='crate',
                    constraint='tangent_to_ground_plane'
                )
            ]
        )

        num_models = dict(
            crate_ball=4,
            crate_cuboid=4,
            crate_cylinder=4
        )

        generator.add_engine(
            tag='fill_crate',
            engine_name='random_pose',
            models=list(num_models.keys()),
            max_num=num_models,
            model_picker='random',
            no_collision=True,
            policies=[
                dict(
                    models=list(num_models.keys()),
                    config=[
                        dict(
                            dofs=['x', 'y'],
                            tag='workspace',
                            workspace='crate_base'
                        ),
                        dict(
                            dofs=['z'],
                            tag='uniform',
                            mean=0.5,
                            min=0.0,
                            max=10.0
                        ),
                        dict(
                            dofs=['roll', 'pitch', 'yaw'],
                            tag='uniform',
                            mean=0,
                            min=-3.141592653589793,
                            max=3.141592653589793
                        )
                    ]
                )
            ]
        )

        crate_model = generator.run(group_name='full_crate')

        self.assertIsNotNone(crate_model)

        n_models = 1
        for tag in num_models:
            n_models += num_models[tag]

        self.assertEqual(crate_model.n_models, n_models)


if __name__ == '__main__':
    unittest.main()
