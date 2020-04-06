#!/usr/bin/env python
# Copyright (c) 2020 - The Procedural Generation for Gazebo authors
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
import random
from pcg_gazebo.generators import WorldGenerator
from pcg_gazebo.generators.creators import box
from pcg_gazebo.generators.occupancy import generate_occupancy_grid


STATIC_CYL = dict(
    tag='static_cylinder',
    description=dict(
        type='cylinder',
        args=dict(
            length="max(0.1, 0.5 * __import__('numpy').random.random())",
            radius="max(0.1, 0.5 * __import__('numpy').random.random())",
            name='cylinder',
            color='xkcd'
        )
    )
)

DYN_BOXES = dict(
    tag='dyn_box',
    description=dict(
        type='box',
        args=dict(
            size="0.5 * __import__('numpy').random.random(3)",
            name='cuboid',
            mass="max(0.1, __import__('numpy').random.random())",
            color='xkcd'
        )
    )
)

BOX_FLOOR_MODEL = box(
    size=[1, 1, 0.01],
    mass=0,
    name='box_floor'
)
BOX_FLOOR_MODEL.name = 'box_floor'
BOX_FLOOR_MODEL.static = True

TANG_TO_GROUND = dict(
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

WORKSPACE = dict(
    name='ws',
    type='workspace',
    frame='world',
    geometry_type='area',
    points=[
        [-30, -30, 0],
        [-30, 30, 0],
        [30, 30, 0],
        [30, -30, 0],
    ]
)


class TestOccupancyGrid(unittest.TestCase):
    def test_gen_map(self):
        policy = dict(
            models=['static_cylinder', 'dyn_box'],
            config=[
                dict(
                    dofs=['x', 'y'],
                    policy=dict(
                        name='workspace',
                        args='ws'
                    )
                )
            ]
        )

        for _ in range(3):
            world_gen = WorldGenerator()
            world_gen.add_asset(**STATIC_CYL)
            self.assertTrue(
                world_gen.assets.is_factory_input('static_cylinder'))

            world_gen.add_asset(**DYN_BOXES)
            self.assertTrue(world_gen.assets.is_factory_input('dyn_box'))

            world_gen.add_asset(
                tag='box_floor', description=BOX_FLOOR_MODEL)
            self.assertTrue(world_gen.assets.is_model('box_floor'))

            world_gen.add_constraint(**TANG_TO_GROUND)
            self.assertIn(
                'tangent_to_ground_plane',
                list(world_gen.constraints.tags))
            world_gen.add_constraint(**WORKSPACE),
            self.assertIn('ws', world_gen.constraints.tags)

            n_cylinders = random.randint(1, 3)
            n_boxes = random.randint(1, 3)
            # Add ground place
            world_gen.add_engine(
                engine_name='fixed_pose',
                tag='gp_engine',
                models=['box_floor'],
                poses=[[0 for _ in range(6)]])
            world_gen.set_model_as_ground_plane('box_floor')

            world_gen.add_engine(
                tag='cyl_engine',
                engine_name='random_pose',
                models=['static_cylinder', 'dyn_box'],
                max_num=dict(
                    static_cylinder=n_cylinders,
                    dyn_box=n_boxes),
                model_picker='random',
                no_collision=True,
                policies=[policy],
                constraints=[
                    dict(
                        model='static_cylinder',
                        constraint='tangent_to_ground_plane'),
                    dict(
                        model='dyn_box',
                        constraint='tangent_to_ground_plane')
                ]
            )

            self.assertTrue(world_gen.run_engines())
            self.assertEqual(len(world_gen.world.models),
                             n_cylinders + n_boxes + 1)

            # First occupancy grid with box floor
            occupancy_output = generate_occupancy_grid(
                world_gen.world.models,
                z_levels=None,
                x_limits=None,
                y_limits=None,
                z_limits=None,
                n_processes=1,
                mesh_type='collision',
                ground_plane_models=['box_floor'])

            self.assertIsNotNone(occupancy_output)
            self.assertEqual(len(occupancy_output['non_static']), n_boxes)
            self.assertEqual(len(occupancy_output['static']), n_cylinders + 1)

            filtered_models = dict()
            for tag in world_gen.world.models:
                if tag != 'box_floor':
                    filtered_models[tag] = world_gen.world.models[tag]

            # Without box floor
            occupancy_output = generate_occupancy_grid(
                filtered_models,
                z_levels=None,
                x_limits=None,
                y_limits=None,
                z_limits=None,
                n_processes=1,
                mesh_type='collision')

            self.assertIsNotNone(occupancy_output)
            self.assertEqual(len(occupancy_output['non_static']), n_boxes)
            self.assertEqual(len(occupancy_output['static']), n_cylinders)

            del world_gen


if __name__ == '__main__':
    unittest.main()
