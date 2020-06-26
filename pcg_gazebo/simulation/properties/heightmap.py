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
from skimage.io import imread, imsave
from .texture import Texture
from ...path import Path
from ...utils import is_array, is_scalar, is_string, \
    is_integer, is_boolean
from ...log import PCG_ROOT_LOGGER
from ...parsers.sdf import create_sdf_element


class Heightmap(object):
    def __init__(self, uri=None, size=[1, 1, 1], pos=[0, 0, 0],
                 use_terrain_paging=False, sampling=2, textures=None,
                 blends=None):
        assert is_array(size), \
            'Input size vector is not an array'
        assert len(size) == 3, \
            'Input size vector must have 3 elements'
        for elem in size:
            assert is_scalar(elem), \
                'Element in size vector is not a scalar'
            assert elem > 0, \
                'Size vector element must be greater than 0'

        assert is_array(pos), \
            'Input position vector is not an array'
        assert len(pos) == 3, \
            'Input position vector must have 3 elements'
        for elem in pos:
            assert is_scalar(elem), \
                'Element in position vector is not a scalar'

        assert is_boolean(use_terrain_paging), \
            'Use terrain paging flag must be a boolean'

        assert is_integer(sampling), 'Sampling must be an integer'
        assert sampling > 0, 'Sampling must be greater than zero'

        self._size = size
        self._pos = pos
        self._image_uri = None
        self._image = None
        self._use_terrain_paging = bool(use_terrain_paging)
        self._sampling = sampling
        self._textures = list()
        self._blends = list()

        if uri is not None:
            self.load_image(uri)

    @property
    def size(self):
        return self._size

    @size.setter
    def size(self, value):
        assert is_array(value), \
            'Input size vector is not an array'
        assert len(value) == 3, \
            'Input size vector must have 3 elements'
        for elem in value:
            assert is_scalar(elem), \
                'Element in size vector is not a scalar'
            assert elem > 0, \
                'Size vector element must be greater than 0'
        self._size = value

    @property
    def position(self):
        return self._pos

    @position.setter
    def position(self, value):
        assert is_array(value), \
            'Input position vector is not an array'
        assert len(value) == 3, \
            'Input position vector must have 3 elements'
        for elem in value:
            assert is_scalar(elem), \
                'Element in position vector is not a scalar'
        self._pos = value

    @property
    def image_uri(self):
        if self._image_uri is not None:
            return self._image_uri.absolute_uri
        return None

    @image_uri.setter
    def image_uri(self, value):
        assert is_string(value), \
            'Input URI must be a string, provided={}'.format(value)
        self._image_uri = Path(value)
        assert self._image_uri.absolute_uri is not None, \
            'Image URI is invalid, path={}'.format(value)

    @property
    def sampling(self):
        return self._sampling

    @sampling.setter
    def sampling(self, value):
        assert is_integer(value), 'Sampling value must be an integer'
        assert value > 0
        self._sampling = value

    @property
    def use_terrain_paging(self):
        return self._use_terrain_paging

    @use_terrain_paging.setter
    def use_terrain_paging(self, value):
        assert is_boolean(value), 'Flag must be a boolean'
        self._use_terrain_paging = bool(value)

    def load_image(self, uri=None):
        if uri is not None:
            self.image_uri = uri
        assert self._image_uri.absolute_uri is not None, \
            'Image URI is invalid, path={}'.format(uri)
        self._image = imread(self._image_uri.absolute_uri)

    def add_texture(self, **kwargs):
        if 'texture' in kwargs:
            assert isinstance(kwargs['texture'], Texture), \
                'Input must be a simulation.properties.Texture type input'
            self._textures.append(kwargs['texture'])
        else:
            self._textures.append(Texture(**kwargs))

    def add_blend(self, **kwargs):
        if 'min_height' in kwargs and 'fade_dist' in kwargs:
            assert is_scalar(kwargs['min_height']), \
                'Min. height must be a scalar'
            assert is_scalar(kwargs['fade_dist']), \
                'Fade distance must be a scalar'
            self._blends.append(
                dict(
                    min_height=kwargs['min_height'],
                    fade_dist=kwargs['fade_dist']))
        elif 'sdf' in kwargs:
            self._blends.append(
                dict(
                    min_height=kwargs['sdf'].min_height.value,
                    fade_dist=kwargs['sdf'].fade_dist.value))
        else:
            raise ValueError(
                'Invalid inputs to set a blend property set')

    def export(self, filename, folder=None, format='png'):
        assert is_string(filename), \
            'Image filename input must be a string'

        if not filename.endswith('.' + format):
            filename += '.' + format

        if not os.path.isdir(folder):
            PCG_ROOT_LOGGER.error(
                'Export folder does not exist, provided={}'.format(folder))
            return False

        imsave(self._image, os.path.join(folder, filename))
        self.image_uri = os.path.join(folder, filename)
        return True

    @staticmethod
    def from_sdf(sdf):
        assert sdf.xml_element_name == 'heightmap', \
            'Input SDF must be a heightmap'
        use_terrain_paging = False
        if sdf.use_terrain_paging is not None:
            use_terrain_paging = sdf.use_terrain_paging.value
        sampling = 2
        if sdf.sampling is not None:
            sampling = sdf.sampling.value
        position = [0, 0, 0]
        if sdf.pos is not None:
            position = sdf.pos.value
        size = [1, 1, 1]
        if sdf.size is not None:
            size = sdf.size.value
        heightmap = Heightmap(
            uri=sdf.uri.value,
            size=size,
            pos=position,
            use_terrain_paging=use_terrain_paging,
            sampling=sampling)
        if sdf.textures is not None:
            for texture in sdf.textures:
                heightmap.add_texture(texture=Texture.from_sdf(texture))
        if sdf.blends is not None:
            for blend in sdf.blends:
                heightmap.add_blend(
                    min_height=blend.min_height.value,
                    fade_dist=blend.fade_dist.value)

        return heightmap

    def to_sdf(self, mode='visual'):
        sdf = create_sdf_element('heightmap')
        if self._image_uri.model_uri is not None:
            sdf.uri = self._image_uri.model_uri
        elif self._image_uri.file_uri is not None:
            sdf.uri = self._image_uri.file_uri
        sdf.size = self._size
        sdf.pos = self._pos
        sdf.use_terrain_paging = self.use_terrain_paging
        sdf.sampling = self.sampling

        if mode == 'visual':
            if len(self._textures) > 0 or len(self._blends) > 0:
                assert len(self._textures) == len(self._blends) + 1, \
                    'Number of textures must be equal to' \
                    ' the number of blends plus one'
                for blend in self._blends:
                    obj = create_sdf_element('blend')
                    obj.min_height = blend['min_height']
                    obj.fade_dist = blend['fade_dist']
                    sdf.add_blend(blend=obj)

                for texture in self._textures:
                    sdf.add_texture(texture=texture.to_sdf())
        return sdf
