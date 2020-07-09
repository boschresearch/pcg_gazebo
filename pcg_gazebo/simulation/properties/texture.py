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
from skimage.io import imread, imsave
from ...path import Path
from ...utils import is_scalar, is_string, PCG_RESOURCES_ROOT_DIR, \
    generate_random_string
from ...log import PCG_ROOT_LOGGER


class Texture(object):
    def __init__(self, diffuse_image_uri=None, diffuse_image=None,
                 normal_image_uri=None, normal_image=None, size=1,
                 name=None):
        assert is_scalar(size), \
            'Size must be a scalar, provided={}'.format(size)
        assert size > 0, 'Size must be greater than zero'
        self._size = size
        self._diffuse_image_uri = None
        self._normal_image_uri = None
        self._diffuse_image = None
        self._normal_image = None
        if name is None or not is_string(name):
            self._name = generate_random_string(5)
        else:
            self._name = name

        if diffuse_image_uri is not None:
            self.diffuse_image_uri = diffuse_image_uri
        elif diffuse_image is not None and \
                isinstance(diffuse_image, np.ndarray) and \
                len(diffuse_image.shape) >= 2:
            self._diffuse_image = diffuse_image
        if normal_image_uri is not None:
            self.normal_image_uri = normal_image_uri
        elif normal_image is not None and \
                isinstance(normal_image, np.ndarray) and \
                len(normal_image.shape) >= 2:
            self._normal_image = normal_image

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

        imsave(
            os.path.join(folder, diffuse_filename),
            self._diffuse_image,
            check_contrast=False)
        self.diffuse_image_uri = os.path.join(folder, diffuse_filename)
        imsave(
            os.path.join(folder, normal_filename),
            self._normal_image,
            check_contrast=False)
        self.normal_image_uri = os.path.join(folder, normal_filename)
        return True

    @staticmethod
    def from_sdf(sdf):
        return Texture(
            size=sdf.size.value[0],
            diffuse_image_uri=sdf.diffuse.value,
            normal_image_uri=sdf.normal.value)

    def to_sdf(self, sdf_version='1.6', filename=None,
               model_folder=None, copy_resources=False):
        from ...parsers.sdf import create_sdf_element
        if model_folder:
            PCG_ROOT_LOGGER.info('Model folder: {}'.format(model_folder))

        if model_folder is None or not os.path.isdir(model_folder):
            PCG_ROOT_LOGGER.warning(
                'Input resources folder to store heightmaps'
                ' does not exist, using the default {}, '
                'dir={}'.format(PCG_RESOURCES_ROOT_DIR, model_folder))
            folder = os.path.join(
                PCG_RESOURCES_ROOT_DIR, 'materials', 'textures')
        else:
            folder = os.path.join(
                model_folder, 'materials', 'textures')

        if not os.path.isdir(folder):
            os.makedirs(folder)
            PCG_ROOT_LOGGER.info(
                'Folder for heightmap textures created: {}'.format(folder))

        prefix = self._name
        if copy_resources and self._diffuse_image_uri is not None:
            PCG_ROOT_LOGGER.info(
                'Copying diffuse image resource <{}> '
                'to model folder <{}>'.format(
                    self._diffuse_image_uri.absolute_uri, folder))
            from shutil import copyfile
            if not os.path.isdir(folder):
                PCG_ROOT_LOGGER.warning(
                    'Input resources folder to store heightmaps'
                    ' does not exist, heightmaps will not be copied'
                    'dir={}'.format(folder))
            else:
                if not folder.endswith('materials/textures'):
                    folder = os.path.join(folder, 'materials', 'textures')
                    if not os.path.isdir(folder):
                        os.makedirs(folder)

                if self._diffuse_image_uri.absolute_uri.startswith(
                        os.path.abspath(model_folder)):
                    PCG_ROOT_LOGGER.info(
                        'Image already exists in destination '
                        'folder: {}'.format(model_folder))
                else:
                    if filename is None:
                        image_filename = os.path.basename(
                            self._diffuse_image_uri.absolute_uri)
                        if not image_filename.endswith('.png'):
                            image_filename += '.png'
                    old_filename = self._diffuse_image_uri.absolute_uri
                    copyfile(
                        old_filename,
                        os.path.join(folder, image_filename))
                    self._diffuse_image_uri = Path(
                        os.path.join(folder, image_filename))
                    PCG_ROOT_LOGGER.info(
                        'Diffuse image file was copied from {} to {}'.format(
                            old_filename,
                            self._diffuse_image_uri.absolute_uri))
        if copy_resources and self._normal_image_uri is not None:
            PCG_ROOT_LOGGER.info(
                'Copying normal image resource <{}> '
                'to model folder <{}>'.format(
                    self._normal_image_uri.absolute_uri, folder))
            from shutil import copyfile
            if not os.path.isdir(folder):
                PCG_ROOT_LOGGER.warning(
                    'Input resources folder to store heightmaps'
                    ' does not exist, heightmaps will not be copied'
                    'dir={}'.format(folder))
            else:
                if not folder.endswith('materials/textures'):
                    folder = os.path.join(folder, 'materials', 'textures')
                    if not os.path.isdir(folder):
                        os.makedirs(folder)

                if self._normal_image_uri.absolute_uri.startswith(
                        os.path.abspath(model_folder)):
                    PCG_ROOT_LOGGER.info(
                        'Image already exists in destination '
                        'folder: {}'.format(model_folder))
                else:
                    if filename is None:
                        image_filename = os.path.basename(
                            self._normal_image_uri.absolute_uri)
                        if not image_filename.endswith('.png'):
                            image_filename += '.png'
                    old_filename = self._normal_image_uri.absolute_uri
                    copyfile(
                        old_filename,
                        os.path.join(folder, image_filename))
                    self._normal_image_uri = Path(
                        os.path.join(folder, image_filename))
                    PCG_ROOT_LOGGER.info(
                        'Normal image file was copied from {} to {}'.format(
                            old_filename, self._normal_image_uri.absolute_uri))

        if (self._diffuse_image_uri is None and
                self._diffuse_image is not None) or \
                (self._normal_image_uri is None and
                 self._normal_image is not None):
            self.export(
                diffuse_filename=prefix + '_diffuse',
                normal_filename=prefix + '_normal',
                folder=folder,
                format='png')

        assert self.diffuse_image_uri is not None, \
            'No image URI for diffuse image found'
        assert self.normal_image_uri is not None, \
            'No image URI for normal image found'
        sdf = create_sdf_element('texture')
        sdf.size = self._size
        if self._diffuse_image_uri.model_uri is not None:
            sdf.diffuse = self._diffuse_image_uri.model_uri
        elif self._diffuse_image_uri.file_uri is not None:
            sdf.diffuse = self._diffuse_image_uri.file_uri
        else:
            raise ValueError('Diffuse image URI is invalid')

        if self._normal_image_uri.model_uri is not None:
            sdf.normal = self._normal_image_uri.model_uri
        elif self._normal_image_uri.file_uri is not None:
            sdf.normal = self._normal_image_uri.file_uri
        else:
            raise ValueError('Normal image URI is invalid')
        return sdf
