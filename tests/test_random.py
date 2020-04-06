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
from pcg_gazebo.utils import is_scalar, is_array, is_integer
from pcg_gazebo import random


class TestRandom(unittest.TestCase):
    def test_seed(self):
        seeds = [random.randint(100) for _ in range(3)]

        for seed in seeds:
            random.init_random_state(seed)
            self.assertEqual(random.PCG_RANDOM_SEED, seed)

            # Test rand
            random.init_random_state(seed)
            ref = [random.rand() for _ in range(5)]

            for _ in range(3):
                random.init_random_state(seed)
                self.assertEqual(random.PCG_RANDOM_SEED, seed)
                self.assertEqual(ref, [random.rand() for _ in range(5)])

            random.init_random_state(seed)
            self.assertEqual(random.PCG_RANDOM_SEED, seed)

            # Test randint
            ref = [random.randint(10) for _ in range(5)]

            for _ in range(3):
                random.init_random_state(seed)
                self.assertEqual(random.PCG_RANDOM_SEED, seed)
                self.assertEqual(ref, [random.randint(10) for _ in range(5)])

            # Test choice
            values = [random.rand() for _ in range(10)]
            random.init_random_state(seed)
            ref = [random.choice(values) for _ in range(5)]
            for _ in range(3):
                random.init_random_state(seed)
                self.assertEqual(random.PCG_RANDOM_SEED, seed)
                self.assertEqual(
                    ref,
                    [random.choice(values) for _ in range(5)])

    def test_rand(self):
        # Test default output
        output = random.rand()
        self.assertTrue(is_scalar(output))
        self.assertGreaterEqual(output, 0)
        self.assertLessEqual(output, 1)

    def test_randint(self):
        # Test default output
        output = random.randint(5)
        self.assertTrue(is_integer(output))
        self.assertGreaterEqual(output, 0)
        self.assertLessEqual(output, 5)

    def test_choice(self):
        # Test default output
        values = [random.rand() for _ in range(10)]
        output = random.choice(values)
        self.assertIn(output, values)

    def test_uniform(self):
        # Test default output
        output = random.uniform()
        self.assertTrue(is_scalar(output))
        self.assertGreaterEqual(output, 0)
        self.assertLessEqual(output, 1)

        # Test input as *args for low and high limits
        output = random.uniform(-10, 10)
        self.assertTrue(is_scalar(output))
        self.assertGreaterEqual(output, -10)
        self.assertLessEqual(output, 10)

        # Test input as *args for low and high
        # limits and size
        output = random.uniform(-5, 5, 3)
        self.assertTrue(is_array(output))
        for item in output:
            self.assertGreaterEqual(item, -5)
            self.assertLessEqual(item, 5)

        # Using keywords
        output = random.uniform(low=-5, high=5, size=3)
        self.assertTrue(is_array(output))
        for item in output:
            self.assertGreaterEqual(item, -5)
            self.assertLessEqual(item, 5)


if __name__ == '__main__':
    unittest.main()
