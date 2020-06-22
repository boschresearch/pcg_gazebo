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
import sys
import numpy as np
from pcg_gazebo import random
from pcg_gazebo.simulation import SimulationModel, ModelGroup
from pcg_gazebo.simulation.physics import ODE, Simbody, Bullet
from pcg_gazebo.generators import WorldGenerator
from pcg_gazebo.generators.creators import box_factory
from pcg_gazebo.utils import generate_random_string


BOX_MODEL = box_factory(
    size=[
        [0.1, 0.1, 0.1]
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

BOX_FACTORY_INPUT = dict(
    type='factory',
    tag='box_factory',
    description=dict(
        type='box',
        args=dict(
            size=[1, 1, 1],
            mass=1,
            name='box'
        )
    )
)

FIXED_ENGINE = dict(
    tag='add_fixed_models',
    engine_name='fixed_pose',
    models=['box'],
    poses=[
        [0, 0, 0, 0, 0, 0]
    ]
)

RANDOM_ENGINE = dict(
    tag='add_random_objects',
    engine_name='random_pose',
    models=[
        'box',
        'box_floor'
    ],
    model_picker='random',
    max_area=0.9,
    no_collision=True,
    max_num=dict(
        box=2,
        box_floor=2
    ),
    policies=[
        dict(
            models=[
                'box',
                'box_floor'
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

TEST_MODEL_GROUP_GENERATOR = dict(
    type='model_generator',
    tag='box_generated',
    description=dict(
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
)

EXAMPLES_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..',
    'examples',
    'world_generator',
    'worlds'
)


class TestWorldGenerator(unittest.TestCase):
    def test_multiple_engines(self):
        generator = WorldGenerator()
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
        self.assertIn(box_name, generator.assets.tags)

        cyl_name = 'cyl_' + generate_random_string(5)
        random_cyl = dict(
            type='cylinder',
            args=dict(
                radius="__import__('pcg_gazebo').random.rand()",
                length="__import__('pcg_gazebo').random.rand()",
                name=cyl_name
            )
        )
        self.assertTrue(generator.add_asset(
            random_cyl,
            tag=cyl_name,
            type='factory'))
        self.assertIn(cyl_name, generator.assets.tags)

        sphere_name = 'sphere_' + generate_random_string(5)
        random_sphere = dict(
            type='sphere',
            args=dict(
                radius="__import__('pcg_gazebo').random.rand()",
                name=sphere_name
            )
        )
        self.assertTrue(generator.add_asset(
            random_sphere,
            tag=sphere_name,
            type='factory'))
        self.assertIn(sphere_name, generator.assets.tags)

        workspace_name = generate_random_string(5)
        self.assertTrue(generator.add_constraint(
            name=workspace_name,
            type='workspace',
            geometry_type='circle',
            radius=100,
            center=[0, 0])
        )

        models = [box_name, cyl_name, sphere_name]
        total_models = 0
        for item in models:
            max_num = dict()
            max_num[item] = random.randint(1, 3)
            total_models += max_num[item]
            self.assertTrue(
                generator.add_engine(
                    tag=item + '_engine',
                    engine_name='random_pose',
                    models=[item],
                    model_picker='random',
                    no_collision=True,
                    max_num=max_num,
                    policies=[
                        dict(
                            models=[item],
                            config=[
                                dict(
                                    dofs=['x', 'y'],
                                    tag='workspace',
                                    workspace=workspace_name
                                )
                            ]
                        )
                    ]
                )
            )
        self.assertTrue(generator.run_engines())
        self.assertEqual(len(generator.world.models), total_models)

    def test_init_from_world_sdf(self):
        generator = WorldGenerator(
            world_file=os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'worlds',
                'test_pcg_example.world'))
        self.assertIsNotNone(generator.world)
        self.assertIsNotNone(generator.world.n_models, 2)
        self.assertIsNotNone(generator.world.n_lights, 1)
        self.assertIn('box', generator.world.models)
        self.assertIn('ground_plane', generator.world.models)
        self.assertIn('sun', generator.world.lights)

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

        workspace_name = generate_random_string(5)
        self.assertTrue(generator.add_constraint(
            name=workspace_name,
            type='workspace',
            geometry_type='circle',
            radius=100,
            center=[0, 0])
        )

        num_models = dict()
        num_models[box_name] = random.randint(2, 5)

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
                        )
                    ]
                )
            ]
        ))

        generator.run_engines(attach_models=True)

        self.assertEqual(generator.world.n_models, num_models[box_name] + 2)
        self.assertIn('box', generator.world.models)
        self.assertIn('ground_plane', generator.world.models)
        self.assertIn('sun', generator.world.lights)

    def test_generator_seed(self):
        if sys.version_info[0] < 3:
            return
        generator = WorldGenerator()

        # Set random generator's seed
        generator.seed = random.randint(low=0, high=10000)

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

        generator.run_engines()
        ref = generator.world.copy()
        self.assertIsNotNone(ref)
        self.assertEqual(ref.n_models, total_num_models)

        for i in range(3):
            generator.run_engines()
            self.assertIsNotNone(generator.world)
            self.assertEqual(ref, generator.world)

    def test_full_config_from_dict(self):
        wg_config = dict(
            assets=dict(
                assets=[
                    dict(description=BOX_MODEL),
                    dict(description=BOX_FLOOR_MODEL),
                    BOX_FACTORY_INPUT,
                    TEST_MODEL_GROUP_GENERATOR
                ],
                ground_plane=['box_floor']
            ),
            engines=[
                dict(
                    tag='add_box_factory',
                    engine_name='fixed_pose',
                    models=['box_factory'],
                    poses=[[0, 0, 0, 0, 0, 0]]),
                dict(
                    tag='add_box_floor',
                    engine_name='fixed_pose',
                    models=['box_floor'],
                    poses=[[1, 1, 1, 0, 0, 0]]),
                dict(
                    tag='add_model_group_generator',
                    engine_name='fixed_pose',
                    models=['box_generated'],
                    poses=[[10, 10, 10, 0, 0, 0]]),
                dict(
                    tag='add_boxes',
                    engine_name='random_pose',
                    models=[
                        'box'
                    ],
                    model_picker='random',
                    max_area=0.9,
                    no_collision=True,
                    max_num=dict(
                        box=5
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
                                        args='boxes_workspace'
                                    )
                                )
                            ]
                        )
                    ]
                ),
                dict(
                    tag='add_generatored_model_groups_random',
                    engine_name='random_pose',
                    models=[
                        'box_generated'
                    ],
                    model_picker='random',
                    max_area=0.9,
                    no_collision=True,
                    max_num=dict(
                        box_generated=1
                    ),
                    policies=[
                        dict(
                            models=[
                                'box_generated'
                            ],
                            config=[
                                dict(
                                    dofs=['x', 'y'],
                                    policy=dict(
                                        name='workspace',
                                        args='generated_boxes_workspace'
                                    )
                                ),
                                dict(
                                    dofs=['z', 'roll', 'pitch', 'yaw'],
                                    policy=dict(
                                        name='value',
                                        args=0
                                    )
                                )
                            ]
                        )
                    ]
                )
            ],
            constraints=[
                dict(
                    name='boxes_workspace',
                    type='workspace',
                    frame='world',
                    geometry_type='area',
                    points=[
                        [-6, -4, 0],
                        [-3, -4, 0],
                        [-3, 0, 0],
                        [-6, 0, 0]
                    ]),
                dict(
                    name='generated_boxes_workspace',
                    type='workspace',
                    frame='world',
                    geometry_type='area',
                    points=[
                        [-100, -100, 0],
                        [100, -100, 0],
                        [100, 100, 0],
                        [-100, 100, 0]
                    ]
                )
            ]
        )

        generator = WorldGenerator()
        generator.from_dict(wg_config)

        self.assertTrue(generator.is_asset('box'))
        self.assertTrue(generator.assets.is_model('box'))
        model = generator.get_asset('box')
        self.assertIsNotNone(model)
        self.assertIsInstance(model, SimulationModel)
        self.assertEqual(model.name, 'box')

        self.assertTrue(generator.is_asset('box_floor'))
        self.assertTrue(generator.assets.is_model('box_floor'))
        model = generator.get_asset('box_floor')
        self.assertIsNotNone(model)
        self.assertIsInstance(model, SimulationModel)
        self.assertEqual(model.name, 'box_floor')

        self.assertTrue(generator.is_asset('box_factory'))
        self.assertTrue(generator.assets.is_factory_input('box_factory'))
        model = generator.get_asset('box_factory')
        self.assertIsNotNone(model)
        self.assertIsInstance(model, SimulationModel)
        self.assertEqual(model.name, 'box_factory')

        self.assertTrue(generator.is_asset('box_generated'))
        self.assertTrue(
            generator.assets.is_model_group_generator('box_generated'))
        model = generator.get_asset('box_generated')
        self.assertIsNotNone(model)
        self.assertIsInstance(model, ModelGroup)
        self.assertEqual(model.name, 'box_generated')
        self.assertEqual(model.n_models, 5)

        for engine in wg_config['engines']:
            self.assertTrue(generator.engines.has_element(engine['tag']))

        for constraint in wg_config['constraints']:
            self.assertTrue(
                generator.engines.has_constraint(
                    constraint['name']))

        self.assertTrue(generator.run_engines())

        # Test if all models where generated
        self.assertEqual(generator.world.n_models, 17)
        self.assertEqual(len(generator.world.model_groups), 3)
        self.assertIn('default', generator.world.model_groups)
        self.assertIn('box_generated', generator.world.model_groups)
        self.assertIn('box_generated_1', generator.world.model_groups)

        # Test export to world SDF file
        self.assertEqual(
            generator.export_world(
                '/tmp',
                'test.world'),
            '/tmp/test.world')
        self.assertTrue(os.path.isfile('/tmp/test.world'))

        # Test export to mesh
        export_formats = ['obj']
        for f in export_formats:
            for mesh_type in ['visual', 'collision']:
                filename = mesh_type + generate_random_string(5)
                generator.world.export_as_mesh(
                    format=f,
                    mesh_type=mesh_type,
                    filename=filename,
                    folder='/tmp')
                self.assertTrue(
                    os.path.isfile(os.path.join('/tmp', filename + '.' + f)))
                self.assertGreater(
                    os.path.getsize(
                        os.path.join('/tmp', filename + '.' + f)), 0)

    def test_add_assets(self):
        generator = WorldGenerator()
        generator.add_asset(BOX_MODEL)
        self.assertTrue(generator.is_asset('box'))
        self.assertTrue(generator.assets.is_model('box'))
        model = generator.get_asset('box')
        self.assertIsNotNone(model)
        self.assertIsInstance(model, SimulationModel)
        self.assertEqual(model.name, 'box')

        generator.add_asset(BOX_FLOOR_MODEL)
        self.assertTrue(generator.is_asset('box_floor'))
        self.assertTrue(generator.assets.is_model('box_floor'))
        model = generator.get_asset('box_floor')
        self.assertIsNotNone(model)
        self.assertIsInstance(model, SimulationModel)
        self.assertEqual(model.name, 'box_floor')

        generator.add_asset(**BOX_FACTORY_INPUT)
        self.assertTrue(generator.is_asset('box_factory'))
        self.assertTrue(generator.assets.is_factory_input('box_factory'))
        model = generator.get_asset('box_factory')
        self.assertIsNotNone(model)
        self.assertIsInstance(model, SimulationModel)
        self.assertEqual(model.name, 'box_factory')

        generator.add_asset(**TEST_MODEL_GROUP_GENERATOR)
        self.assertTrue(generator.is_asset('box_generated'))
        self.assertTrue(
            generator.assets.is_model_group_generator('box_generated'))
        model = generator.get_asset('box_generated')
        self.assertIsNotNone(model)
        self.assertIsInstance(model, ModelGroup)
        self.assertEqual(model.name, 'box_generated')
        print(model.to_sdf(type='model'))
        self.assertEqual(model.n_models, 5)

    def test_add_assets_from_dict(self):
        generator = WorldGenerator()

        config = dict(
            assets=dict(
                assets=[
                    BOX_FACTORY_INPUT
                ],
                ground_plane=['box_factory']
            )
        )

        generator.from_dict(config)

        self.assertTrue(generator.is_asset('box_factory'))
        self.assertTrue(generator.assets.is_factory_input('box_factory'))
        model = generator.get_asset('box_factory')
        self.assertIsNotNone(model)
        self.assertIsInstance(model, SimulationModel)
        self.assertEqual(model.name, 'box_factory')

    def test_set_physics_engine(self):
        generator = WorldGenerator()
        generator.set_physics_engine(ODE())
        self.assertIsNotNone(generator.world.physics)
        self.assertEqual(generator.world.physics.engine, 'ode')

        generator = WorldGenerator()
        generator.set_physics_engine(Bullet())
        self.assertIsNotNone(generator.world.physics)
        self.assertEqual(generator.world.physics.engine, 'bullet')

        generator = WorldGenerator()
        generator.set_physics_engine(Simbody())
        self.assertIsNotNone(generator.world.physics)
        self.assertEqual(generator.world.physics.engine, 'simbody')

    def test_set_physics_engines_args(self):
        generator = WorldGenerator()
        generator.set_physics_engine(
            'ode',
            max_step_size=0.001,
            real_time_factor=1,
            real_time_update_rate=1000,
            max_contacts=20,
            min_step_size=0.0001,
            iters=50,
            sor=1.3,
            type='quick',
            precon_iters=0,
            use_dynamic_moi_rescaling=False,
            friction_model='pyramid_model',
            cfm=0,
            erp=0.2,
            contact_surface_layer=0.001,
            contact_max_correcting_vel=100)

        self.assertIsNotNone(generator.world.physics)
        self.assertEqual(generator.world.physics.engine, 'ode')

        generator = WorldGenerator()
        generator.set_physics_engine(
            'bullet',
            max_step_size=0.001,
            real_time_factor=1,
            real_time_update_rate=1000,
            max_contacts=20,
            min_step_size=0.0001,
            iters=50,
            sor=1.3,
            cfm=0,
            erp=0.2,
            contact_surface_layer=0.001,
            split_impulse=True,
            split_impulse_penetration_threshold=-0.01)

        self.assertIsNotNone(generator.world.physics)
        self.assertEqual(generator.world.physics.engine, 'bullet')

        generator = WorldGenerator()
        generator.set_physics_engine(
            'simbody',
            max_step_size=0.001,
            real_time_factor=1,
            real_time_update_rate=1000,
            max_contacts=20,
            min_step_size=0.0001,
            accuracy=0.001,
            max_transient_velocity=0.01,
            stiffness=1e8,
            dissipation=100,
            plastic_coef_restitution=0.5,
            plastic_impact_velocity=0.5,
            static_friction=0.9,
            dynamic_friction=0.9,
            viscous_friction=0,
            override_impact_capture_velocity=0.001,
            override_stiction_transition_velocity=0.001)

        self.assertIsNotNone(generator.world.physics)
        self.assertEqual(generator.world.physics.engine, 'simbody')

    def test_set_physics_engine_from_dict(self):
        config = dict(
            physics=dict(
                engine='ode',
                args=dict(
                    max_step_size=0.001,
                    real_time_factor=1,
                    real_time_update_rate=1000,
                    max_contacts=20,
                    min_step_size=0.0001,
                    iters=50,
                    sor=1.3,
                    type='quick',
                    precon_iters=0,
                    use_dynamic_moi_rescaling=False,
                    friction_model='pyramid_model',
                    cfm=0,
                    erp=0.2,
                    contact_surface_layer=0.001,
                    contact_max_correcting_vel=100
                )
            )
        )

        generator = WorldGenerator()
        generator.from_dict(config)
        self.assertIsNotNone(generator.world.physics)
        self.assertEqual(generator.world.physics.engine, 'ode')

        config = dict(
            physics=dict(
                engine='bullet',
                args=dict(
                    max_step_size=0.001,
                    real_time_factor=1,
                    real_time_update_rate=1000,
                    max_contacts=20,
                    min_step_size=0.0001,
                    iters=50,
                    sor=1.3,
                    cfm=0,
                    erp=0.2,
                    contact_surface_layer=0.001,
                    split_impulse=True,
                    split_impulse_penetration_threshold=-0.01
                )
            )
        )

        generator = WorldGenerator()
        generator.from_dict(config)
        self.assertIsNotNone(generator.world.physics)
        self.assertEqual(generator.world.physics.engine, 'bullet')

        config = dict(
            physics=dict(
                engine='simbody',
                args=dict(
                    max_step_size=0.001,
                    real_time_factor=1,
                    real_time_update_rate=1000,
                    max_contacts=20,
                    min_step_size=0.0001,
                    accuracy=0.001,
                    max_transient_velocity=0.01,
                    stiffness=1e8,
                    dissipation=100,
                    plastic_coef_restitution=0.5,
                    plastic_impact_velocity=0.5,
                    static_friction=0.9,
                    dynamic_friction=0.9,
                    viscous_friction=0,
                    override_impact_capture_velocity=0.001,
                    override_stiction_transition_velocity=0.001
                )
            )
        )

        generator = WorldGenerator()
        generator.from_dict(config)
        self.assertIsNotNone(generator.world.physics)
        self.assertEqual(generator.world.physics.engine, 'simbody')

    def test_add_model(self):
        generator = WorldGenerator()

        name = generate_random_string(5)
        obj = SimulationModel(name)
        obj.add_cuboid_link(
            link_name='link',
            size=[random.rand() for _ in range(3)],
            mass=random.rand())

        pose = [random.rand() for _ in range(6)]
        generator.add_model(obj, [pose])

        self.assertTrue(generator.assets.is_model(name))
        self.assertTrue(generator.run_engines())

        self.assertIn(name, generator.world.models)

    def test_example_bouncing_balls(self):
        gen_config_file = os.path.join(EXAMPLES_DIR, 'bouncing_balls_ode.yml')

        generator = WorldGenerator()
        generator.from_yaml(gen_config_file)

        self.assertTrue(generator.run_engines())
        self.assertGreaterEqual(len(generator.world.models), 1)

    def test_example_empty_world_bullet(self):
        gen_config_file = os.path.join(EXAMPLES_DIR, 'empty_world_bullet.yml')

        generator = WorldGenerator()
        generator.from_yaml(gen_config_file)

        self.assertTrue(generator.run_engines())
        self.assertGreaterEqual(len(generator.world.models), 1)

    def test_example_empty_world_ode(self):
        gen_config_file = os.path.join(EXAMPLES_DIR, 'empty_world_ode.yml')

        generator = WorldGenerator()
        generator.from_yaml(gen_config_file)

        self.assertTrue(generator.run_engines())
        self.assertGreaterEqual(len(generator.world.models), 1)

    def test_example_full_crates_ode(self):
        gen_config_file = os.path.join(EXAMPLES_DIR, 'full_crates_ode.yml')

        generator = WorldGenerator()
        generator.from_yaml(gen_config_file)

        self.assertTrue(generator.run_engines())
        self.assertGreaterEqual(len(generator.world.models), 1)

    def test_example_random_workspaces_ode(self):
        gen_config_file = os.path.join(
            EXAMPLES_DIR, 'random_workspaces_ode.yml')

        generator = WorldGenerator()
        generator.from_yaml(gen_config_file)

        self.assertTrue(generator.run_engines())
        self.assertGreaterEqual(len(generator.world.models), 1)


if __name__ == '__main__':
    unittest.main()
