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
import sys
import unittest
from pcg_gazebo import random
from pcg_gazebo.utils import generate_random_string
from pcg_gazebo.collection_managers import AssetsManager
from pcg_gazebo.generators.creators import box
from pcg_gazebo.generators.item_pickers import create_picker


class TestItemPicker(unittest.TestCase):
    def test_roulette_picker_no_max_num(self):
        if sys.version_info.major == 2:
            return
        max_num = dict()
        fitness = dict()
        counter = dict()
        for i in range(5):
            name = generate_random_string(5)
            max_num[name] = None
            fitness[name] = random.randint(1, 10)
            counter[name] = 0

        picker = create_picker(
            tag='roulette',
            items=list(max_num.keys()),
            max_num=max_num,
            fitness=fitness)
        self.assertIsNotNone(picker)

        total_items = 100

        for _ in range(total_items):
            output = picker.get_selection()
            self.assertIsNotNone(output)
            self.assertIn(output, counter.keys())
            counter[output] += 1

    def test_roulette_picker_with_max_num(self):
        if sys.version_info.major == 2:
            return
        max_num = dict()
        fitness = dict()
        counter = dict()
        total_items = 0
        for i in range(5):
            name = generate_random_string(5)
            max_num[name] = random.randint(1, 10)
            fitness[name] = random.randint(1, 10)
            counter[name] = 0

            total_items += max_num[name]

        picker = create_picker(
            tag='roulette',
            items=list(max_num.keys()),
            max_num=max_num,
            fitness=fitness)
        self.assertIsNotNone(picker)

        output = picker.get_selection()

        while output is not None:
            self.assertIsNotNone(output)
            self.assertIn(output, counter.keys())
            counter[output] += 1

            temp_counter = 0
            for tag in counter:
                temp_counter += counter[tag]
            self.assertLessEqual(temp_counter, total_items)
            output = picker.get_selection()

        temp_counter = 0
        for tag in counter:
            temp_counter += counter[tag]
        self.assertEqual(temp_counter, total_items)

    def test_size_picker(self):
        if sys.version_info.major == 2:
            return
        assets_manager = AssetsManager.get_instance()

        assets_manager.add(
            tag='box_1',
            description=box(size=[10, 10, 10])
        )

        assets_manager.add(
            tag='box_2',
            description=box(size=[5, 5, 5])
        )

        assets_manager.add(
            tag='box_3',
            description=box(size=[1, 1, 1])
        )

        picker = create_picker(
            tag='size',
            items=['box_{}'.format(i + 1) for i in range(3)],
            max_num=dict(
                box_1=1,
                box_2=1,
                box_3=1
            ))
        self.assertIsNotNone(picker)

        self.assertEqual(picker.get_selection(), 'box_1')
        self.assertEqual(picker.get_selection(), 'box_2')
        self.assertEqual(picker.get_selection(), 'box_3')
        self.assertIsNone(picker.get_selection())

    def test_random_picker(self):
        if sys.version_info.major == 2:
            return
        max_num = dict()
        counter = dict()
        total_items = 0
        for i in range(5):
            name = generate_random_string(5)
            max_num[name] = random.randint(1, 10)
            counter[name] = 0

            total_items += max_num[name]

        picker = create_picker(
            tag='random',
            items=list(max_num.keys()),
            max_num=max_num)
        self.assertIsNotNone(picker)

        output = picker.get_selection()

        while output is not None:
            self.assertIsNotNone(output)
            self.assertIn(output, counter.keys())
            counter[output] += 1

            temp_counter = 0
            for tag in counter:
                temp_counter += counter[tag]
            self.assertLessEqual(temp_counter, total_items)
            output = picker.get_selection()

        temp_counter = 0
        for tag in counter:
            temp_counter += counter[tag]
        self.assertEqual(temp_counter, total_items)


if __name__ == '__main__':
    unittest.main()
