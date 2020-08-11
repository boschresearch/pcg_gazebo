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
from __future__ import print_function
import os
import numpy as np
from skimage.io import imread, imsave
from skimage.color import rgb2gray
from skimage import transform
from .biomes import Biome
from ..simulation.properties import Heightmap
from ..utils import is_array, is_string
from ..log import PCG_ROOT_LOGGER
from .. import random
try:
    from noise import pnoise2, snoise2
    HAS_NOISE_SUPPORT = True
except Exception as ex:
    print(ex)
    HAS_NOISE_SUPPORT = False


class HeightmapGenerator(object):
    def __init__(self, image_size=[2**6 + 1, 2**6 + 1], texture_size=1,
                 map_size=[10, 10, 1], position=[0, 0, 0],
                 name='heightmap', sampling=2, use_terrain_paging=False):
        assert is_array(image_size), \
            'Image size input must be a vector'
        assert image_size[0] > 0 and image_size[1] > 0, \
            'Image size coordinates must be greater than zero'
        assert is_array(map_size), \
            'Map size input must be a vector'
        assert len(map_size) == 3, \
            'Map size vector must have 3 components'
        assert sum([i > 0 for i in map_size]) == 3, \
            'Map size components must be greater than zero'
        assert is_array(position), \
            'Position input must be a vector'
        assert len(position) == 3, \
            'Position vector must have 3 components'
        assert texture_size > 0, 'Size must be greater than zero'
        assert is_string(name), 'Heightmap name is invalid, ' \
            'value={}'.format(name)
        assert len(name) > 0, 'Name cannot me an empty string'
        self._layers = list()
        self._masks = list()
        self._image_size = image_size
        self._texture_size = texture_size
        self._map_size = map_size
        self._position = position
        self._textures = list()
        self._name = name
        self._biome = None
        self._moisture_zone = 0
        self._use_terrain_paging = use_terrain_paging
        self._sampling = sampling
        self._heightmap = Heightmap(
            pos=position,
            size=map_size,
            use_terrain_paging=use_terrain_paging,
            sampling=sampling)

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        assert is_string(value), 'Heightmap name is invalid, ' \
            'value={}'.format(value)
        self._name = value

    @property
    def heightmap_image(self):
        image = np.zeros(self._image_size)
        for layer in self._layers:
            image += layer
        return np.multiply(image, self.mask)

    @property
    def heightmap(self):
        self._heightmap.image = self.heightmap_image
        self._heightmap.position = self._position
        self._heightmap.size = self._map_size
        if self.biome is not None:
            self.biome.set_biome_to_heightmap(self._heightmap)
        return self._heightmap

    @property
    def mask(self):
        image = np.ones(self._image_size)
        for mask in self._masks:
            image = np.multiply(image, mask)
        return image

    @property
    def texture_size(self):
        return self._texture_size

    @property
    def image_size(self):
        return self._image_size

    @property
    def map_size(self):
        return self._heightmap.size

    @map_size.setter
    def map_size(self, value):
        self._heightmap.size = value

    @property
    def position(self):
        return self._heightmap.position

    @position.setter
    def position(self, value):
        self._heightmap.position = value

    @property
    def biome(self):
        return self._biome

    @biome.setter
    def biome(self, value):
        if isinstance(value, Biome):
            self._biome = value
        elif hasattr(value, 'xml_element_name'):
            self._biome = Biome.from_sdf(value)
        else:
            raise ValueError('Invalid biome input')

    def _resize(self, image):
        return transform.resize(
            image, output_shape=self._image_size) * 255

    def _open_file(self, filename):
        assert is_string(filename), \
            'Input filename must be a string'
        assert os.path.isfile(filename), \
            'Invalid image filename, value={}'.format(
                filename)
        image = imread(filename)
        assert image is not None, 'Could not load image'
        if len(image.shape) == 3:
            image = rgb2gray(image)
        return image

    def add_layer_from_file(self, filename):
        image = self._open_file(filename)
        self._layers.append(image)

    def add_mask_from_file(self, filename):
        image = self._open_file(filename)
        self._masks.append(image)

    def add_custom_mask(self, mask):
        assert isinstance(mask, np.ndarray), \
            'Input image must be a numpy array'
        assert len(mask.shape) == 2, \
            'Input image must be a 2D array'
        self._masks.append(transform.resize(
            mask, output_shape=self._image_size))

    def add_custom_layer(self, custom_image):
        assert isinstance(custom_image, np.ndarray), \
            'Input image must be a numpy array'
        assert len(custom_image.shape) == 2, \
            'Input image must be a 2D array'
        image = self._resize(custom_image)
        self._layers.append(image)

    def add_zero_normal_layer(self):
        self._layers.append(np.zeros(self._image_size))

    def add_random_layer(self, scale=1, min_value=0, max_value=1):
        image = self.get_random_noise(scale, min_value, max_value)
        self._layers.append(image)

    def add_perlin_noise_layer(self, freq=16.0, octaves=1):
        image = self.get_perlin_noise(freq, octaves)
        self._layers.append(image)

    def add_simplex_noise_layer(self, freq=16.0, octaves=1):
        image = self.get_simplex_noise(freq, octaves)
        self._layers.append(image)

    def _perlin_noise(self, freq=1.0, octaves=1, noise_fcn='perlin'):
        assert HAS_NOISE_SUPPORT, 'No noise generation support'
        assert freq > 0, 'Frequency must be greater than 0'
        assert octaves > 0, 'Octaves must be greater than 0'
        image = np.zeros(self._image_size)
        if noise_fcn == 'perlin':
            noise_fcn = pnoise2
        elif noise_fcn == 'simplex':
            noise_fcn = snoise2
        else:
            raise NotImplementedError(
                'Noise algorithm is invalid, value={}'.format(
                    noise_fcn))
        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                image[i, j] = noise_fcn(
                    i / (freq * octaves), j / (freq * octaves), octaves)

        image = (image - image.min()) * 256 / (image.max() - image.min())
        return image.astype(np.uint8)

    def get_simplex_noise(self, freq=1.0, octaves=1):
        image = self._perlin_noise(freq, octaves, 'simplex')
        return image

    def get_perlin_noise(self, freq=1.0, octaves=1):
        image = self._perlin_noise(freq, octaves, 'perlin')
        return image

    def get_random_noise(self, scale=1, min_value=0, max_value=1):
        image = random.rand(*self._image_size)
        image = (max_value - min_value) * image + min_value
        image *= 256.0
        image[np.nonzero(image > 256)[0]] = 256
        return image.astype(np.uint8)

    def reset(self):
        self._layers = list()
        self._heightmap = Heightmap(
            pos=self._position,
            size=self._map_size,
            use_terrain_paging=self._use_terrain_paging,
            sampling=self._sampling)

    def save_image(self, filename, image=None):
        assert is_string(filename), \
            'Input filename must be a string'
        imsave()

    def show(self, image=None):
        import matplotlib.pyplot as plt
        if image is None:
            image = self.heightmap_image
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.imshow(image, cmap=plt.cm.gray)
        plt.show()

    def show_layer(self, index):
        if len(self._layers) == 0:
            PCG_ROOT_LOGGER.warning('No heightmap layers found')
            return False
        assert index < len(self._layers), \
            'Layer index is out of range'
        self.show(self._layers[index])
        return True

    def show_mask(self, index=None):
        if index is None:
            self.show(self.mask.astype(np.uint8) * 256)
        else:
            if len(self._masks) == 0:
                PCG_ROOT_LOGGER.warning('No mask layers found')
                return False
            assert index < len(self._masks), \
                'Mask index is out of range'
            self.show(self._masks[index].astype(np.uint8) * 256)
        return True

    def as_model(self):
        from ..simulation import SimulationModel
        model = SimulationModel(name=self._name)
        model.add_link(name='link')
        model.links['link'].add_empty_visual(name='visual')
        model.links['link'].visuals[0].geometry.set_heightmap(
            heightmap=self.heightmap)
        model.links['link'].add_empty_collision(name='collision')
        model.links['link'].collisions[0].geometry.set_heightmap(
            heightmap=self.heightmap)
        return model

    @staticmethod
    def from_sdf(sdf):
        assert sdf.xml_element_name == 'heightmap', \
            'Input SDF element must be of type heightmap'
        hg = HeightmapGenerator()
        hg._heightmap = Heightmap.from_sdf(sdf)
        hg._biome = Biome.from_sdf(sdf)
        hg._image_size = hg._heightmap._image.shape
        hg.add_layer_from_file(hg._heightmap.image_uri)
        return hg
