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
from pcg_gazebo.simulation import add_custom_gazebo_resource_path, \
    SimulationModel
from pcg_gazebo.simulation.properties import Heightmap
from pcg_gazebo.generators import HeightmapGenerator

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
        self.assertIsNotNone(sdf.models)
        self.assertEqual(len(sdf.models[0].links[0].visuals), 1)
        self.assertEqual(len(sdf.models[0].links[0].collisions), 1)

        sdf_heightmap = sdf.models[0].links[0].visuals[0].geometry.heightmap
        self.assertIsNotNone(sdf_heightmap)

        uri_path_prefix = \
            'model://pcg_winding_valley_heightmap/materials/textures'
        self.assertIn(
            uri_path_prefix,
            sdf_heightmap.uri.value)
        self.assertEqual(len(sdf_heightmap.textures), 3)
        for texture in sdf_heightmap.textures:
            self.assertIn(uri_path_prefix, texture.diffuse.value)
            self.assertIn(uri_path_prefix, texture.normal.value)
            self.assertEqual(texture.size.value, [10])
        self.assertEqual(len(sdf_heightmap.blends), 2)
        for blend in sdf_heightmap.blends:
            self.assertGreater(blend.min_height.value, 0)
            self.assertGreater(blend.fade_dist.value, 0)
        self.assertEqual(sdf_heightmap.size.value, [1000, 1000, 25])
        self.assertEqual(sdf_heightmap.pos.value, [0, 0, -4])

        sdf_heightmap = \
            sdf.models[0].links[0].collisions[0].geometry.heightmap
        self.assertIsNotNone(sdf_heightmap)

        self.assertIn(
            uri_path_prefix,
            sdf_heightmap.uri.value)
        self.assertEqual(sdf_heightmap.size.value, [1000, 1000, 25])
        self.assertEqual(sdf_heightmap.pos.value, [0, 0, -4])

    def test_from_sdf(self):
        add_custom_gazebo_resource_path(
            os.path.join(CUR_DIR, '..', 'examples', 'models'))

        # Parse the SDF file
        sdf = parse_sdf(
            os.path.join(
                CUR_DIR, '..', 'examples', 'models',
                'pcg_winding_valley_heightmap', 'model.sdf'))
        self.assertIsNotNone(sdf)

        # Convert SDF heightmap element to simulation heightmap element
        sdf_heightmap = sdf.models[0].links[0].visuals[0].geometry.heightmap
        heightmap = Heightmap.from_sdf(sdf_heightmap)
        self.assertIsNotNone(heightmap)

        output_sdf = heightmap.to_sdf()
        self.assertIsNotNone(output_sdf)

        sdf_heightmap = \
            sdf.models[0].links[0].collisions[0].geometry.heightmap
        heightmap = Heightmap.from_sdf(sdf_heightmap)
        self.assertIsNotNone(heightmap)

        output_sdf = heightmap.to_sdf()
        self.assertIsNotNone(output_sdf)

        self.assertEqual(sdf_heightmap.size.value, output_sdf.size.value)
        self.assertEqual(sdf_heightmap.pos.value, output_sdf.pos.value)

        # Convert model to SimulationModel element
        model = SimulationModel.from_sdf(sdf.models[0])
        self.assertIsNotNone(model)
        output_sdf = model.to_sdf()
        self.assertIsNotNone(output_sdf)

        self.assertIsNotNone(output_sdf)
        self.assertEqual(len(output_sdf.links[0].visuals), 1)
        self.assertEqual(len(output_sdf.links[0].collisions), 1)

        sdf_heightmap = output_sdf.links[0].visuals[0].geometry.heightmap
        self.assertIsNotNone(sdf_heightmap)

        uri_path_prefix = \
            'model://pcg_winding_valley_heightmap/materials/textures'
        self.assertIn(
            uri_path_prefix,
            sdf_heightmap.uri.value)
        self.assertEqual(len(sdf_heightmap.textures), 3)
        for texture in sdf_heightmap.textures:
            self.assertIn(uri_path_prefix, texture.diffuse.value)
            self.assertIn(uri_path_prefix, texture.normal.value)
            self.assertEqual(texture.size.value, [10])
        self.assertEqual(len(sdf_heightmap.blends), 2)
        for blend in sdf_heightmap.blends:
            self.assertGreater(blend.min_height.value, 0)
            self.assertGreater(blend.fade_dist.value, 0)
        self.assertEqual(sdf_heightmap.size.value, [1000, 1000, 25])
        self.assertEqual(sdf_heightmap.pos.value, [0, 0, -4])

        sdf_heightmap = \
            output_sdf.links[0].collisions[0].geometry.heightmap
        self.assertIsNotNone(sdf_heightmap)

        self.assertIn(
            uri_path_prefix,
            sdf_heightmap.uri.value)
        self.assertEqual(sdf_heightmap.size.value, [1000, 1000, 25])
        self.assertEqual(sdf_heightmap.pos.value, [0, 0, -4])

    def test_init_heightmap_generator_from_sdf(self):
        add_custom_gazebo_resource_path(
            os.path.join(CUR_DIR, '..', 'examples', 'models'))

        # Parse the SDF file
        sdf = parse_sdf(
            os.path.join(
                CUR_DIR, '..', 'examples', 'models',
                'pcg_winding_valley_heightmap', 'model.sdf'))
        self.assertIsNotNone(sdf)

        sdf_heightmap = sdf.models[0].links[0].visuals[0].geometry.heightmap
        hg = HeightmapGenerator.from_sdf(sdf_heightmap)
        self.assertIsNotNone(hg)


if __name__ == '__main__':
    unittest.main()
