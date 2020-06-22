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
import os
import unittest
from pcg_gazebo.parsers import parse_sdf
from pcg_gazebo.simulation import add_custom_gazebo_resource_path

CUR_DIR = os.path.dirname(os.path.abspath(__file__))


class TestHeightmap(unittest.TestCase):
    def test_parse_heightmap_model(self):
        add_custom_gazebo_resource_path(
            os.path.join(CUR_DIR, '..', 'examples', 'models'))

        # Parse the SDF file
        sdf = parse_sdf(
            os.path.join(
                CUR_DIR, '..', 'examples', 'models',
                'pcg_winding_valley_heightmap', 'model.sdf'))
        self.assertIsNotNone(sdf)


if __name__ == '__main__':
    unittest.main()
