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
import numpy as np
from pcg_gazebo.utils import generate_random_string
from pcg_gazebo.generators import ModelGroupGenerator
from pcg_gazebo.generators.creators import box_factory
from pcg_gazebo import random


BOX_MODEL = box_factory(
    size=[
        [1, 1, 1]
    ],
    mass=1,
    use_permutation=True,
    name='box'
)[0]
BOX_MODEL.name = 'box_' + generate_random_string(5)

BOX_FLOOR_MODEL = box_factory(
    size=[
        [1, 1, 0.01]
    ],
    mass=1,
    use_permutation=True,
    name='box_floor'
)[0]
BOX_FLOOR_MODEL.name = 'box_floor_' + generate_random_string(5)

FIXED_ENGINE = dict(
    tag='add_fixed_models',
    engine_name='fixed_pose',
    models=[BOX_FLOOR_MODEL.name],
    poses=[
        [0, 0, 0, 0, 0, 0]
    ]
)

NUM_BOXES = dict()
NUM_BOXES[BOX_MODEL.name] = 2

RANDOM_ENGINE = dict(
    tag='add_random_objects',
    engine_name='random_pose',
    models=[BOX_MODEL.name],
    model_picker='random',
    max_area=0.9,
    no_collision=True,
    max_num=NUM_BOXES,
    policies=[
        dict(
            models=[BOX_MODEL.name],
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


class TestModelGroupGenerator(unittest.TestCase):
    def test_generator_seed(self):
        if sys.version_info[0] < 3:
            return
        generator = ModelGroupGenerator()
        # Set random generator's seed
        generator.seed = np.random.randint(low=0, high=10000)

        box_name = 'box_' + generate_random_string(5)
        random_box = dict(
            type='box',
            args=dict(
                size="__import__('pcg_gazebo').random.rand(3)",
                mass="__import__('pcg_gazebo').random.rand()",
                name=box_name
            )
        )
        self.assertTrue(generator.add_asset(
            random_box,
            tag=box_name,
            type='factory'))

        cylinder_name = 'cyl_' + generate_random_string(5)
        random_cyl = dict(
            type='cylinder',
            args=dict(
                radius="__import__('pcg_gazebo').random.rand()",
                length="__import__('pcg_gazebo').random.rand()",
                mass="__import__('pcg_gazebo').random.rand()",
                name=cylinder_name
            )
        )
        self.assertTrue(generator.add_asset(random_cyl, tag=cylinder_name))

        sphere_name = 'sphere_' + generate_random_string(5)
        random_sphere = dict(
            type='sphere',
            args=dict(
                radius="__import__('pcg_gazebo').random.rand()",
                mass="__import__('pcg_gazebo').random.rand()",
                name=sphere_name
            )
        )
        self.assertTrue(generator.add_asset(random_sphere, tag=sphere_name))

        workspace_name = generate_random_string(5)
        self.assertTrue(generator.add_constraint(
            name=workspace_name,
            type='workspace',
            geometry_type='circle',
            radius=100,
            center=[0, 0]))

        num_models = dict()
        num_models[box_name] = random.randint(1, 3)
        num_models[cylinder_name] = random.randint(1, 3)
        num_models[sphere_name] = random.randint(1, 3)

        total_num_models = 0
        for tag in num_models:
            total_num_models += num_models[tag]

        engine_name = generate_random_string(5)
        self.assertTrue(generator.add_engine(
            tag=engine_name,
            engine_name='random_pose',
            models=list(num_models.keys()),
            model_picker='random',
            no_collision=True,
            max_num=num_models,
            policies=[
                dict(
                    models=list(num_models.keys()),
                    config=[
                        dict(
                            dofs=['x', 'y'],
                            tag='workspace',
                            workspace=workspace_name
                        ),
                        dict(
                            dofs=['roll', 'pitch', 'yaw'],
                            tag='uniform',
                            min=-2 * np.pi,
                            max=2 * np.pi
                        )
                    ]
                )
            ]
        ))

        ref = generator.run('test')
        self.assertIsNotNone(ref)
        self.assertEqual(ref.n_models, total_num_models)

        for _ in range(3):
            model = generator.run('test')
            self.assertIsNotNone(model)
            self.assertEqual(
                ref.to_sdf('model'),
                model.to_sdf('model')
            )

    def test_add_engines_and_constraints_as_obj(self):
        # Add constraints, engines and assets individually
        generator = ModelGroupGenerator()
        self.assertTrue(generator.add_constraint(**TANGENT_CONSTRAINT))
        self.assertTrue(generator.add_constraint(**WORKSPACE_CONSTRAINT))
        self.assertTrue(generator.add_asset(BOX_FLOOR_MODEL))
        self.assertTrue(generator.add_asset(BOX_MODEL))
        self.assertTrue(generator.add_engine(**FIXED_ENGINE))
        self.assertTrue(generator.add_engine(**RANDOM_ENGINE))

        self.assertTrue(generator.is_asset(BOX_FLOOR_MODEL.name))
        self.assertTrue(generator.is_asset(BOX_MODEL.name))

        generator.set_model_as_ground_plane(BOX_FLOOR_MODEL.name)

        # Run all engines and retrieve model group
        group = generator.run('test')
        self.assertIsNotNone(group)
        self.assertEqual(group.n_models, 3)

    def test_add_engines_and_constraints_as_dict(self):
        # Add as configuration
        config = dict(
            assets=dict(
                ground_plane=[BOX_FLOOR_MODEL.name],
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

        self.assertTrue(generator.is_asset(BOX_FLOOR_MODEL.name))
        self.assertTrue(generator.is_asset(BOX_MODEL.name))

        group = generator.run('test')
        print(group.to_sdf())
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
            geometry_type='area',
            points=[
                [-0.5, -0.4, 0],
                [-0.5, 0.4, 0],
                [0.5, 0.4, 0],
                [0.5, -0.4, 0]
            ]
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
