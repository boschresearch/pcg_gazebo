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
import numpy as np
from tabulate import tabulate
from skimage.io import imread, imsave
from ...log import PCG_ROOT_LOGGER
from ...utils import is_string, is_array, is_integer, \
    PCG_RESOURCES_ROOT_DIR
from ...path import Path
from ...simulation.properties import Heightmap


class Biome(object):
    FLAT_NORMAL_COLOR = [129, 148, 250]

    def __init__(self, n_moisture_zones=6,
                 n_elevation_zones=4, biomes=None,
                 image_size=[100, 100], size=1):
        assert is_integer(n_moisture_zones) and \
            n_moisture_zones > 0, \
            'Number of moisture zones must be '\
            'greater than zero'
        assert is_integer(n_elevation_zones) and \
            n_elevation_zones > 0, \
            'Number of elevation zones must be ' \
            'greater than zero'
        assert is_integer(size) and size > 0, \
            'Size must be greater than zero'
        self._n_moisture_zones = n_moisture_zones
        self._n_elevation_zones = n_elevation_zones
        self._image_size = image_size
        self._size = size
        self._moisture_zone = 0

        self._biomes = dict()
        self._rules = list()

        self._min_heights = dict()
        self._fade_dist = dict()
        for i in range(1, self._n_elevation_zones):
            self._fade_dist[i] = 5
            self._min_heights[i] = i * 10

    def __str__(self):
        headers = ['Moisture zone #{}'.format(i)
                   for i in range(self._n_moisture_zones)]
        table = list()
        for e in range(self._n_elevation_zones):
            line = ['Elevation zone #{}'.format(e)]
            if e in self._min_heights:
                line[0] += ' (Min. height: {} m)'.format(
                    self._min_heights[e])
            for m in range(self._n_moisture_zones):
                biome = self.get_biome(e, m)
                if biome is None:
                    b = '-'
                else:
                    b = biome
                line.append(b)
            table.append(line)
        return str(tabulate(table, headers=headers))

    @property
    def n_moisture_zones(self):
        return self._n_moisture_zones

    @n_moisture_zones.setter
    def n_moisture_zones(self, value):
        assert is_integer(value) and value > 0, \
            'Number of moisture zones must be '\
            'greater than zero'
        self._n_moisture_zones = value

    @property
    def n_elevation_zones(self):
        return self._n_elevation_zones

    @n_elevation_zones.setter
    def n_elevation_zones(self, value):
        assert is_integer(value) and value > 0, \
            'Number of elevation zones must be '\
            'greater than zero'
        self._n_elevation_zones = value

    @property
    def moisture_zone(self):
        return self._moisture_zone

    @moisture_zone.setter
    def moisture_zone(self, value):
        assert is_integer(value) and \
            value < self._n_moisture_zones, \
            'Invalid moisture zone index'
        self._moisture_zone = value

    def _get_flat_color_image(self, color):
        image = np.ones(
            (self._image_size[0], self._image_size[1], 3),
            dtype=np.uint8)
        for i in range(3):
            image[:, :, i] = \
                color[i] * image[:, :, i]
        return image

    def _load_image(self, image):
        if is_string(image):
            image_path = Path(image)
            assert image_path.absolute_uri is not None, \
                'Path to image is invalid, value={}'.format(
                    image)
            return imread(image_path.absolute_uri)
        elif isinstance(image, np.ndarray):
            return image
        else:
            return image

    def save_image(self, folder=None, moisture_zone=0,
                   elevation_zone=0):
        if folder is None:
            output_folder = os.path.join(
                PCG_RESOURCES_ROOT_DIR,
                'materials',
                'textures'
            )
            if not os.path.isdir(output_folder):
                os.makedirs(output_folder)
        else:
            output_folder = folder
            assert os.path.isdir(output_folder), \
                'Provided folder does not exist, provided={}'.format(
                    output_folder)
        biome = self.get_biome(elevation_zone, moisture_zone)
        if biome is None:
            PCG_ROOT_LOGGER.warning(
                'No biome found for moisture zone #{} '
                'and elevation zone #{}'.format(
                    moisture_zone, elevation_zone))
            return None

        full_filename_diffuse = os.path.join(
            output_folder,
            '{}_elevation_{}_moisture_{}_diffuse.png'.format(
                biome, elevation_zone, moisture_zone))
        full_filename_normal = os.path.join(
            output_folder,
            '{}_elevation_{}_moisture_{}_normal.png'.format(
                biome, elevation_zone, moisture_zone))

        imsave(
            full_filename_diffuse,
            self.get_diffuse(biome),
            check_contrast=False)

        imsave(
            full_filename_normal,
            self.get_normal(biome),
            check_contrast=False)

        return dict(
            diffuse=full_filename_diffuse,
            normal=full_filename_normal)

    def get_diffuse(self, biome):
        if biome not in self._biomes:
            PCG_ROOT_LOGGER.warning(
                'Biome {} does not exist'.format(biome))
            return None
        if 'color' in self._biomes[biome]:
            image = self._get_flat_color_image(
                self._biomes[biome]['color'])
        else:
            image = self._biomes[biome]['diffuse']
        return image

    def get_normal(self, biome):
        if biome not in self._biomes:
            PCG_ROOT_LOGGER.warning(
                'Biome {} does not exist'.format(biome))
            return None
        return self._biomes[biome]['normal']

    def add_biome(self, name, size=10, **kwargs):
        assert is_string(name), 'Name of biome must be a string'
        assert is_integer(size) and size > 0, \
            'Size must be greater than zero'
        if 'color' in kwargs:
            assert is_array(kwargs['color']), \
                'Color input must be an array'
            assert len(kwargs['color']) == 3, \
                'Color vector must have 3 components'
            for c in kwargs['color']:
                assert c >= 0 and c <= 255, \
                    'Color component must be 0 <= c <= 255'
            self._biomes[name] = dict(color=kwargs['color'])

            self._biomes[name]['normal'] = \
                self._get_flat_color_image(
                    self.FLAT_NORMAL_COLOR)
        if 'diffuse' in kwargs:
            diffuse = self._load_image(kwargs['diffuse'])
            if diffuse is None:
                raise ValueError(
                    'Invalid diffuse image input, type={}'.format(
                        type(kwargs['diffuse'])))
            self._biomes[name] = dict(diffuse=diffuse)
            if 'normal' in kwargs:
                normal = self._load_image(kwargs['normal'])
                if normal is None:
                    raise ValueError(
                        'Invalid normal image input, type={}'.format(
                            type(kwargs['normal'])))
                self._biomes[name]['normal'] = normal
            else:
                self._biomes[name]['normal'] = \
                    self._get_flat_color_image(
                        self.FLAT_NORMAL_COLOR)

        self._biomes[name]['size'] = size

    def add_rule(self, biome, moisture_zone, elevation_zone):
        assert moisture_zone < self._n_moisture_zones, \
            'Moisture zone index is out of range, it should ' \
            'be less than {}'.format(self._n_moisture_zones)
        assert elevation_zone < self._n_elevation_zones, \
            'Elevation zone index is out of range, it should ' \
            'be less than {}'.format(self._n_elevation_zones)
        assert biome in self._biomes, 'Biome <{}> is invalid'.format(
            biome)

        rule = dict(
            elevation=elevation_zone,
            moisture=moisture_zone)
        rule['biome'] = biome
        if self.get_biome(elevation_zone, moisture_zone) is None:
            self._rules.append(rule)
            return True
        else:
            PCG_ROOT_LOGGER.info(
                'Rule for <{}> biome for elevation zone #{} and'
                ' moisture zone #{} already exists'.format(
                    biome, moisture_zone, elevation_zone))
            return False

    def get_biome(self, elevation_zone, moisture_zone):
        for rule in self._rules:
            if rule['elevation'] == elevation_zone and \
                    rule['moisture'] == moisture_zone:
                return rule['biome']
        return None

    def set_min_height(self, height, elevation_zone):
        assert is_integer(elevation_zone) and \
            elevation_zone > 0, \
            'Elevation zone #0 does not need a ' \
            'min. height'
        assert elevation_zone > 0, \
            'Elevation zone index must be greater than zero'
        self._min_heights[elevation_zone] = height

    def reset_min_height(self):
        self._min_heights = dict()

    def get_min_height(self, elevation_zone):
        if elevation_zone not in self._min_heights:
            return None
        else:
            return self._min_heights[elevation_zone]

    def set_fade_dist(self, fade_dist, elevation_zone):
        assert fade_dist >= 0, \
            'Fade distance must be greater or' \
            ' equal to zero'
        assert elevation_zone > 0, \
            'Elevation zone index must be greater than zero'
        if elevation_zone in self._fade_dist:
            self._fade_dist[elevation_zone] = fade_dist

    def get_fade_dist(self, elevation_zone):
        if elevation_zone not in self._fade_dist:
            return None
        else:
            return self._fade_dist[elevation_zone]

    @staticmethod
    def from_sdf(sdf):
        assert sdf.xml_element_name == 'heightmap', \
            'The SDF to be parsed must be a heightmap'
        if sdf.textures is not None:
            if len(sdf.textures) > 1:
                assert sdf.blends is not None, \
                    'For multiple heightmap textures, ' \
                    '<blend> elements are also expected'
            biome = Biome(1, len(sdf.textures))
            for i in range(len(sdf.textures)):
                diffuse_path = Path(sdf.textures[i].diffuse.value)
                normal_path = Path(sdf.textures[i].normal.value)
                biome.add_biome(
                    'biome_{}'.format(i),
                    size=sdf.textures[i].size.value[0],
                    diffuse=diffuse_path.absolute_uri,
                    normal=normal_path.absolute_uri)
                biome.add_rule('biome_{}'.format(i), 0, i)
            biome._min_heights = dict()
            if len(sdf.textures) > 1:
                for i in range(len(sdf.blends)):
                    biome.set_min_height(
                        sdf.blends[i].min_height.value, i + 1)
                    biome.set_fade_dist(
                        sdf.blends[i].fade_dist.value, i + 1)
            return biome
        else:
            return None

    def show(self):
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(
            self._n_elevation_zones, self._n_moisture_zones,
            sharex=True, sharey=True, squeeze=True, figsize=(20, 15))

        def _plot_texture(_ax, _e, _m):
            biome = self.get_biome(_e, _m)
            _ax.imshow(self.get_diffuse(biome))
            _ax.set_title('Biome: {}'.format(biome), fontsize=8)
            _ax.tick_params(
                axis='both', bottom=False,
                labelbottom=False, left=False,
                labelleft=False)
            _ax.set_xlabel('Moisture zone #{}'.format(_m), fontsize=6)
            x_label = 'Elevation zone #{}'.format(_e)
            if _e in self._min_heights:
                x_label += ' (Min. height: {} m)'.format(self._min_heights[_e])
            _ax.set_ylabel(x_label, fontsize=6)
            return _ax

        if self._n_moisture_zones > 1 and self._n_elevation_zones > 1:
            for e in range(self._n_elevation_zones):
                for m in range(self._n_moisture_zones):
                    axs[e, m] = _plot_texture(axs[e, m], e, m)
        elif self._n_moisture_zones == 1 and self._n_elevation_zones > 1:
            for e in range(self._n_elevation_zones):
                axs[e] = _plot_texture(axs[e], e, 0)
        elif self._n_moisture_zones == 1 and self._n_elevation_zones == 1:
            axs = _plot_texture(axs, 0, 0)
        return fig

    def set_biome_to_heightmap(self, heightmap):
        assert isinstance(heightmap, Heightmap), 'Input must be a heightmap'
        heightmap.reset()
        for e in range(self._n_elevation_zones):
            biome = self.get_biome(e, self._moisture_zone)
            diffuse_image = self.get_diffuse(biome)
            normal_image = self.get_normal(biome)
            heightmap.add_texture(
                diffuse_image=diffuse_image,
                normal_image=normal_image,
                size=self._size,
                name=biome
            )

            if e > 0:
                min_height = self.get_min_height(e)
                fade_dist = self.get_fade_dist(e)
                if min_height is None or fade_dist is None:
                    fade_dist = 5
                    min_height = e * 10
                heightmap.add_blend(
                    min_height=min_height,
                    fade_dist=fade_dist)
