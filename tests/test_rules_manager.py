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
from pcg_gazebo.generators.rules import create_rule, get_rule_parameters
from pcg_gazebo.generators import ConstraintsManager, RulesManager
<<<<<<< HEAD
=======
from pcg_gazebo.utils import generate_random_string
>>>>>>> c8a1b369afa2e91f68883776ce7f10026d3e3dcf
import numpy as np


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


DOF_TAGS = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']


class TestRulesManager(unittest.TestCase):
    def test_examples(self):
        sample = get_rule_parameters('value')
        self.assertIn('dofs', sample)
        self.assertIn('value', sample)

        sample = get_rule_parameters('from_set')
        self.assertIn('dofs', sample)
        self.assertIn('values', sample)

        sample = get_rule_parameters('random')
        self.assertIn('dofs', sample)
        self.assertIn('scaling_factor', sample)
        self.assertIn('offset', sample)

        sample = get_rule_parameters('uniform')
        self.assertIn('dofs', sample)
        self.assertIn('mean', sample)
        self.assertIn('min', sample)
        self.assertIn('max', sample)

        sample = get_rule_parameters('workspace')
        self.assertIn('dofs', sample)
        self.assertIn('workspace', sample)

    def test_fixed_value_rule(self):
        value = random.random()
        rule = create_rule('value', value=value)

        self.assertIsNotNone(rule)
        self.assertEqual(rule.value, value)

        for tag in rule.dofs:
            self.assertFalse(rule.dofs[tag])

        for tag in DOF_TAGS:
            dofs = dict()
            for t in DOF_TAGS:
                dofs[t] = t == tag
            rule.dofs = dofs
            pose = rule.get_pose()
            for t in DOF_TAGS:
                if t == tag:
                    self.assertTrue(np.isclose(getattr(pose, t), value))
                else:
                    self.assertEqual(getattr(pose, t), 0)

    def test_from_set_rule(self):
        values = [random.random() for _ in range(5)]
        rule = create_rule('from_set', values=values)

        self.assertIsNotNone(rule)
        self.assertEqual(rule.values, values)

        for tag in rule.dofs:
            self.assertFalse(rule.dofs[tag])

        for tag in DOF_TAGS:
            dofs = dict()
            for t in DOF_TAGS:
                dofs[t] = t == tag
            rule.dofs = dofs
            pose = rule.get_pose()
            for t in DOF_TAGS:
                if t == tag:
                    found_value = False
                    for v in values:
                        if np.isclose(v, getattr(pose, t)):
                            found_value = True
                            break

                    self.assertTrue(found_value)
                else:
                    self.assertEqual(getattr(pose, t), 0)

    def test_random_rule(self):
        scaling_factor = random.random()
        offset = random.random()

        rule = create_rule(
            'random',
            scaling_factor=scaling_factor,
            offset=offset)

        self.assertIsNotNone(rule)
        self.assertEqual(rule.scaling_factor, scaling_factor)
        self.assertEqual(rule.offset, offset)

        for tag in rule.dofs:
            self.assertFalse(rule.dofs[tag])

        for tag in DOF_TAGS:
            dofs = dict()
            for t in DOF_TAGS:
                dofs[t] = t == tag
            rule.dofs = dofs
            pose = rule.get_pose()
            for t in DOF_TAGS:
                if t == tag:
                    self.assertNotEqual(getattr(pose, t), 0)
                else:
                    self.assertEqual(getattr(pose, t), 0)

    def test_uniform_rule(self):
        mean = random.random()
        min = mean - 1
        max = mean + 1

        rule = create_rule(
            'uniform',
            mean=mean,
            min=min,
            max=max)

        self.assertIsNotNone(rule)
        self.assertEqual(rule.mean, mean)
        self.assertEqual(rule.min, min),
        self.assertEqual(rule.max, max)

        for tag in rule.dofs:
            self.assertFalse(rule.dofs[tag])

        for tag in DOF_TAGS:
            dofs = dict()
            for t in DOF_TAGS:
                dofs[t] = t == tag
            rule.dofs = dofs
            pose = rule.get_pose()
            for t in DOF_TAGS:
                if t == tag:
                    self.assertGreaterEqual(getattr(pose, t), min)
                    self.assertLessEqual(getattr(pose, t), max)
                else:
                    print(pose.position)
                    self.assertEqual(getattr(pose, t), 0)

    def test_within_workspace_rule(self):
        cm = ConstraintsManager.get_instance()
        self.assertTrue(cm.add(**WORKSPACE_CONSTRAINT))
        self.assertIn(WORKSPACE_CONSTRAINT['name'], cm.tags)

        rule = create_rule(
            'workspace',
            workspace=WORKSPACE_CONSTRAINT['name'])

        self.assertIsNotNone(rule)

        pose = rule.get_pose()
        constraint = cm.get(WORKSPACE_CONSTRAINT['name'])
        self.assertTrue(constraint.contains_point([pose.x, pose.y]))

        rule.dofs = dict(x=True, y=True, z=True)
        pose = rule.get_pose()
        constraint = cm.get(WORKSPACE_CONSTRAINT['name'])
        self.assertTrue(constraint.contains_point([pose.x, pose.y]))

    def test_add_rule_to_manager(self):
        rm = RulesManager.get_instance()

<<<<<<< HEAD
        mean = random.random()
        min = mean - 1
        max = mean + 1

        rule = create_rule(
            'uniform',
            dofs=dict(x=True, y=True),
            mean=mean,
            min=min,
            max=max)

        rm.add(name='uniform_rule', rule_obj=rule)
        self.assertIn('uniform_rule', rm.tags)
=======
        value = random.random()
        rule = create_rule('value', value=value)

        name = generate_random_string(5)
        self.assertTrue(rm.add(name, rule))
        self.assertIn(name, rm.tags)

        name = generate_random_string(5)
        self.assertTrue(rm.add(name, type='value', value=value))
        self.assertIn(name, rm.tags)
>>>>>>> c8a1b369afa2e91f68883776ce7f10026d3e3dcf


if __name__ == '__main__':
    unittest.main()
