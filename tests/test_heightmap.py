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
import sys
import unittest
import shutil
import numpy as np
from pcg_gazebo.utils import generate_random_string
from pcg_gazebo.path import Path
from pcg_gazebo.parsers import parse_sdf
from pcg_gazebo.simulation import add_custom_gazebo_resource_path, \
    SimulationModel
from pcg_gazebo.simulation.properties import Heightmap
from pcg_gazebo.generators import HeightmapGenerator
from pcg_gazebo.generators.biomes import Biome, WhittakerBiome

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
        self.assertIsNotNone(hg.biome)

        self.assertEqual(hg.biome.n_elevation_zones, 3)
        self.assertEqual(hg.biome.n_moisture_zones, 1)

        for i in range(hg.biome.n_elevation_zones):
            tag = hg.biome.get_biome(i, 0)
            self.assertIsNotNone(tag)
            self.assertIsNotNone(hg.biome.get_diffuse(tag))
            self.assertIsNotNone(hg.biome.get_normal(tag))

    def test_heightmap_generator_to_sdf(self):
        hg = HeightmapGenerator()
        model = hg.as_model()

        self.assertIsNotNone(model)
        self.assertEqual(model.n_links, 1)
        self.assertIn('link', model.links)

        self.assertEqual(len(model.links['link'].visuals), 1)
        self.assertEqual(
            model.links['link'].visuals[0].geometry.geo_type,
            'heightmap')
        self.assertEqual(len(model.links['link'].collisions), 1)
        self.assertEqual(
            model.links['link'].collisions[0].geometry.geo_type,
            'heightmap')

        model_sdf = model.to_sdf()
        self.assertIsNotNone(model_sdf)
        heightmap = model_sdf.links[0].visuals[0].geometry.heightmap
        self.assertIsNotNone(heightmap.uri)
        self.assertIn('.pcg', heightmap.uri.value)
        self.assertIn('file://', heightmap.uri.value)
        self.assertIn('/materials/textures/', heightmap.uri.value)

        image_path = Path(heightmap.uri.value)
        self.assertTrue(os.path.isfile(image_path.absolute_uri))

        # Clean up files
        os.remove(image_path.absolute_uri)

    def test_heightmap_generator_to_gazebo_model(self):
        hg = HeightmapGenerator()
        hg.biome = WhittakerBiome()
        model_name = generate_random_string(10)
        output_dir = os.path.join(
            os.path.expanduser('~'), '.gazebo', 'models')
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)

        model = hg.as_model()
        model.name = model_name

        model.to_gazebo_model(
            output_dir=output_dir,
            copy_resources=True)

        self.assertTrue(
            os.path.isdir(os.path.join(output_dir, model_name)))

        sdf = parse_sdf(os.path.join(output_dir, model_name, 'model.sdf'))
        self.assertIsNotNone(sdf)
        self.assertIsNotNone(sdf.models[0])

        heightmap = \
            sdf.models[0].links[0].visuals[0].geometry.heightmap
        self.assertIsNotNone(heightmap.uri)
        self.assertIn(
            'model://{}'.format(model_name), heightmap.uri.value)
        self.assertIn('/materials/textures/', heightmap.uri.value)
        self.assertTrue(heightmap.uri.value.endswith('.png'))

        heightmap = \
            sdf.models[0].links[0].collisions[0].geometry.heightmap
        self.assertIsNotNone(heightmap.uri)
        self.assertIn(
            'model://{}'.format(model_name), heightmap.uri.value)
        self.assertIn('/materials/textures/', heightmap.uri.value)
        self.assertTrue(heightmap.uri.value.endswith('.png'))

        shutil.rmtree(os.path.join(output_dir, model_name))

    def test_store_perlin_noise_map(self):
        if sys.version_info[0] < 3:
            return
        hg = HeightmapGenerator(image_size=[5, 5])
        hg.add_perlin_noise_layer()
        ref_image = hg.heightmap_image
        self.assertIsNotNone(ref_image)
        self.assertEqual(list(ref_image.shape), hg.image_size)
        self.assertGreater(np.nonzero(ref_image != 0)[0].size, 0)

        model = hg.as_model()

        model_sdf = model.to_sdf()
        self.assertIsNotNone(model_sdf)
        heightmap = model_sdf.links[0].visuals[0].geometry.heightmap
        self.assertIsNotNone(heightmap.uri)
        self.assertIn('.pcg', heightmap.uri.value)
        self.assertIn('file://', heightmap.uri.value)
        self.assertIn('/materials/textures/', heightmap.uri.value)

        image_path = Path(heightmap.uri.value)
        self.assertTrue(os.path.isfile(image_path.absolute_uri))

        # Clean up files
        os.remove(image_path.absolute_uri)

    def test_store_simplex_noise_map(self):
        if sys.version_info[0] < 3:
            return
        hg = HeightmapGenerator(image_size=[5, 5])
        hg.add_simplex_noise_layer()
        ref_image = hg.heightmap_image
        self.assertIsNotNone(ref_image)
        self.assertEqual(list(ref_image.shape), hg.image_size)
        self.assertGreater(np.nonzero(ref_image != 0)[0].size, 0)

        model = hg.as_model()

        model_sdf = model.to_sdf()
        self.assertIsNotNone(model_sdf)
        heightmap = model_sdf.links[0].visuals[0].geometry.heightmap
        self.assertIsNotNone(heightmap.uri)
        self.assertIn('.pcg', heightmap.uri.value)
        self.assertIn('file://', heightmap.uri.value)
        self.assertIn('/materials/textures/', heightmap.uri.value)

        image_path = Path(heightmap.uri.value)
        self.assertTrue(os.path.isfile(image_path.absolute_uri))

        # Clean up files
        os.remove(image_path.absolute_uri)

    def test_store_random_noise_map(self):
        if sys.version_info[0] < 3:
            return
        hg = HeightmapGenerator(image_size=[5, 5])
        hg.add_random_layer()
        ref_image = hg.heightmap_image
        self.assertIsNotNone(ref_image)
        self.assertEqual(list(ref_image.shape), hg.image_size)
        self.assertGreater(np.nonzero(ref_image != 0)[0].size, 0)

        model = hg.as_model()

        model_sdf = model.to_sdf()
        self.assertIsNotNone(model_sdf)
        heightmap = model_sdf.links[0].visuals[0].geometry.heightmap
        self.assertIsNotNone(heightmap.uri)
        self.assertIn('.pcg', heightmap.uri.value)
        self.assertIn('file://', heightmap.uri.value)
        self.assertIn('/materials/textures/', heightmap.uri.value)

        image_path = Path(heightmap.uri.value)
        self.assertTrue(os.path.isfile(image_path.absolute_uri))

        # Clean up files
        os.remove(image_path.absolute_uri)

    def test_biome(self):
        biome = Biome(n_moisture_zones=4, n_elevation_zones=3)
        self.assertTrue(biome.n_elevation_zones, 3)
        self.assertTrue(biome.n_moisture_zones, 4)

    def test_parse_biome_from_heightmap(self):
        add_custom_gazebo_resource_path(
            os.path.join(CUR_DIR, '..', 'examples', 'models'))

        # Parse the SDF file
        sdf = parse_sdf(
            os.path.join(
                CUR_DIR, '..', 'examples', 'models',
                'pcg_winding_valley_heightmap', 'model.sdf'))
        self.assertIsNotNone(sdf)

        sdf_heightmap = sdf.models[0].links[0].visuals[0].geometry.heightmap
        biome = Biome.from_sdf(sdf_heightmap)

        self.assertIsNotNone(biome)
        self.assertEqual(biome.n_elevation_zones, 3)
        self.assertEqual(biome.n_moisture_zones, 1)

        for i in range(biome.n_elevation_zones):
            tag = biome.get_biome(i, 0)
            self.assertIsNotNone(tag)
            self.assertIsNotNone(biome.get_diffuse(tag))
            self.assertIsNotNone(biome.get_normal(tag))

    def test_whittaker_biome(self):
        biome = WhittakerBiome()
        self.assertTrue(biome.n_elevation_zones, 4)
        self.assertTrue(biome.n_moisture_zones, 6)

        for e in range(biome.n_elevation_zones):
            for m in range(biome.n_moisture_zones):
                paths = biome.save_image(
                    moisture_zone=m, elevation_zone=e)
                self.assertTrue(os.path.isfile(paths['diffuse']))
                self.assertTrue(os.path.isfile(paths['normal']))

                diffuse = os.path.basename(paths['diffuse'])
                self.assertTrue(diffuse.endswith('.png'))
                os.remove(paths['diffuse'])

                normal = os.path.basename(paths['normal'])
                self.assertTrue(normal.endswith('.png'))
                os.remove(paths['normal'])


if __name__ == '__main__':
    unittest.main()
