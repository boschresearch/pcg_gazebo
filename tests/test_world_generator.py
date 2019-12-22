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
from pcg_gazebo.simulation import SimulationModel, ModelGroup
from pcg_gazebo.simulation.physics import ODE, Simbody, Bullet
from pcg_gazebo.generators import WorldGenerator
from pcg_gazebo.generators.creators import box_factory


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
        'test_box',
        'box_floor'
    ],
    model_picker='size',
    max_area=0.9,
    no_collision=True,
    max_num=dict(
        test_box=2,
        box_floor=2
    ),
    policies=[
        dict(
            models=[
                'test_box',
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
            model='test_box',
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


class TestWorldGenerator(unittest.TestCase):
    def test_full_config_from_dict(self):
        wg_config = dict(
            assets=dict(
                assets=[
                    dict(description=BOX_MODEL),
                    dict(description=BOX_FLOOR_MODEL),                    
                    BOX_FACTORY_INPUT,
                    TEST_MODEL_GROUP_GENERATOR
                ],
                ground_planecatkin=['box_floor']
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
                    model_picker='size',
                    max_area=0.9,
                    no_collision=True,
                    max_num=dict(
                        box=3
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
                    model_picker='size',
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
                    )         
                ),
                dict(
                    name='generated_boxes_workspace',
                    type='workspace',
                    frame='world',
                    geometry=dict( 
                    type='area',
                    description=dict(
                        points=[
                            [-100, -100, 0],
                            [100, -100, 0],
                            [100, 100, 0],
                            [-100, 100, 0]
                        ]         
                    )
                    )         
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
        self.assertTrue(generator.assets.is_model_group_generator('box_generated'))
        model = generator.get_asset('box_generated')
        self.assertIsNotNone(model)
        self.assertIsInstance(model, ModelGroup)
        self.assertEqual(model.name, 'box_generated')
        self.assertEqual(model.n_models, 5)

        for engine in wg_config['engines']:
            self.assertTrue(generator.engines.has_element(engine['tag']))
        
        for constraint in wg_config['constraints']:
            self.assertTrue(generator.engines.has_constraint(constraint['name']))

        self.assertTrue(generator.run_engines())

        self.assertEqual(generator.world.n_models, 15)
        self.assertEqual(len(generator.world.model_groups), 3)
        self.assertIn('default', generator.world.model_groups)
        self.assertIn('box_generated', generator.world.model_groups)
        self.assertIn('box_generated_1', generator.world.model_groups)
        
        self.assertEqual(generator.export_world('/tmp', 'test.world'), '/tmp/test.world')
        self.assertTrue(os.path.isfile('/tmp/test.world'))

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
        self.assertTrue(generator.assets.is_model_group_generator('box_generated'))
        model = generator.get_asset('box_generated')
        self.assertIsNotNone(model)
        self.assertIsInstance(model, ModelGroup)
        self.assertEqual(model.name, 'box_generated')
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

if __name__ == '__main__':
    unittest.main()