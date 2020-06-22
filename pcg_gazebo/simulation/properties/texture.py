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
from ...path import Path
from ...utils import is_scalar, is_string
from ...log import PCG_ROOT_LOGGER


class Texture(object):
    def __init__(self, diffuse_image_uri=None, normal_image_uri=None, size=1):
        assert is_scalar(size), 'Size must be a scalar'
        assert size > 0, 'Size must be greater than zero'
        self._size = size
        self._diffuse_image_uri = None
        self._normal_image_uri = None
        self._diffuse_image = None
        self._normal_image = None

        if diffuse_image_uri is not None:
            self.diffuse_image_uri = diffuse_image_uri
        if normal_image_uri is not None:
            self.normal_image_uri = normal_image_uri

        self.load()

    @property
    def size(self):
        return self._size

    @size.setter
    def size(self, value):
        assert is_scalar(value), 'Size must be a scalar'
        assert value > 0, 'Size must be greater than zero'
        self._size = value

    @property
    def diffuse_image_uri(self):
        if self._diffuse_image_uri is not None:
            return self._diffuse_image_uri.absolute_uri
        return None

    @diffuse_image_uri.setter
    def diffuse_image_uri(self, value):
        assert is_string(value), 'Input URI must be a string'
        self._diffuse_image_uri = Path(value)
        assert self._diffuse_image_uri.absolute_uri is not None, \
            'Diffuse image URI is invalid, path={}'.format(value)
        self.load()

    @property
    def diffuse(self):
        return self._diffuse_image

    @property
    def normal_image_uri(self):
        if self._normal_image_uri is not None:
            return self._normal_image_uri.absolute_uri
        return None

    @normal_image_uri.setter
    def normal_image_uri(self, value):
        assert is_string(value), 'Input URI must be a string'
        self._normal_image_uri = Path(value)
        assert self._normal_image_uri.absolute_uri is not None, \
            'Normal image URI is invalid, path={}'.format(value)
        self.load()

    @property
    def normal_image(self):
        return self._normal_image

    def load(self):
        if self.diffuse_image_uri is not None:
            self._diffuse_image = imread(self.diffuse_image_uri)
        if self.normal_image_uri is not None:
            self._normal_image = imread(self.normal_image_uri)

    def export(self, diffuse_filename, normal_filename,
               folder=None, format='png'):
        assert is_string(diffuse_filename), \
            'Diffuse image filename input must be a string'

        assert is_string(normal_filename), \
            'Normal image filename input must be a string'

        if not diffuse_filename.endswith('.' + format):
            diffuse_filename += '.' + format
        if not normal_filename.endswith('.' + format):
            normal_filename += '.' + format

        if not os.path.isdir(folder):
            PCG_ROOT_LOGGER.error(
                'Export folder does not exist, provided={}'.format(folder))
            return False

        imsave(self._diffuse_image, os.path.join(folder, diffuse_filename))
        self.diffuse_image_uri = os.path.join(folder, diffuse_filename)
        imsave(self._normal_image, os.path.join(folder, normal_filename))
        self.normal_image_uri = os.path.join(folder, normal_filename)
        return True

    @staticmethod
    def from_sdf(sdf):
        return Texture(
            size=sdf.size.value,
            diffuse_image_uri=sdf.diffuse.value,
            normal_image_uri=sdf.normal.value)

    def to_sdf(self):
        from ...parsers.sdf import create_sdf_element
        assert self.diffuse_image_uri is not None, \
            'No image URI for diffuse image found'
        assert self.normal_image_uri is not None, \
            'No image URI for normal image found'
        sdf = create_sdf_element('texture')
        sdf.size = self._size
        if self._uri.model_uri is not None:
            sdf.diffuse = self._diffuse_image_uri.model_uri
        elif self._uri.file_uri is not None:
            sdf.normal = self._diffuse_image_uri.file_uri
        return sdf
