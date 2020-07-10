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
import trimesh
from shapely.ops import triangulate
from shapely.geometry import MultiPoint
from skimage.io import imread, imsave
from .texture import Texture
from .mesh import Mesh
from ...path import Path
from ...utils import is_array, is_scalar, is_string, \
    is_integer, is_boolean, PCG_RESOURCES_ROOT_DIR
from ...log import PCG_ROOT_LOGGER
from ...parsers.sdf import create_sdf_element


class Heightmap(object):
    def __init__(self, uri=None, size=[1, 1, 1], pos=[0, 0, 0],
                 use_terrain_paging=False, sampling=2, textures=None,
                 blends=None, image=None):
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
        self._mesh = None

        if uri is not None:
            self.load_image(uri)
        elif image is not None:
            assert isinstance(image, np.ndarray), \
                'Input image must be a numpy array'
            assert len(image.shape) == 2, \
                'Input image must be a 2D array'
            self._image = image

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
        self._mesh = None
        self._size = value

    @property
    def mesh(self):
        if self._mesh is None:
            self._mesh = self.as_mesh()
        return self._mesh

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
    def image(self):
        return self._image

    @image.setter
    def image(self, value):
        assert isinstance(value, np.ndarray), \
            'Input image must be a numpy array'
        assert len(value.shape) == 2, \
            'Input image must be a 2D array'
        self._mesh = None
        self._image = value

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

    def reset(self):
        self._textures = list()
        self._blends = list()

    def export(self, filename, folder=None, format='png'):
        assert is_string(filename), \
            'Image filename input must be a string'

        if not filename.endswith('.' + format):
            filename += '.' + format

        if not os.path.isdir(folder):
            PCG_ROOT_LOGGER.error(
                'Export folder does not exist, provided={}'.format(folder))
            return False

        imsave(
            os.path.join(folder, filename),
            self._image, check_contrast=False)
        self.image_uri = os.path.join(folder, filename)
        return True

    def as_mesh(self):
        if self._image is None:
            PCG_RESOURCES_ROOT_DIR.error(
                'No image found for description of heightmap')
            return None

        vertices = np.zeros((np.size(self._image), 3))

        index_x, index_y = np.meshgrid(
            np.linspace(0, self._image.shape[0] - 1, self._image.shape[0]),
            np.linspace(0, self._image.shape[1] - 1, self._image.shape[1])
        )

        vertices[:, 0] = np.reshape(index_x, -1)
        vertices[:, 1] = np.reshape(index_y, -1)
        faces = list()

        polygons = triangulate(
            MultiPoint(
                [(xi, yi) for xi, yi in
                 zip(index_x.flatten(), index_y.flatten())]))

        for i in range(len(polygons)):
            triangle = list()
            for j in range(3):
                point = polygons[i].boundary.coords[j]
                index = np.nonzero(
                    np.logical_and(vertices[:, 0] == point[0],
                                   vertices[:, 1] == point[1]))[0]
                triangle.append(index[0])
            faces.append(triangle)

        faces = np.array(faces)

        vertices[:, 0] = vertices[:, 0] / vertices[:, 0].max() * \
            self._size[0] - self._size[0] / 2 + self._pos[0]
        vertices[:, 1] = vertices[:, 1] / vertices[:, 1].max() * \
            self._size[1] - self._size[1] / 2 + self._pos[1]
        vertices[:, 2] = self._size[2] * \
            np.reshape(self._image / 255.0, -1) + \
            self._pos[2]

        return Mesh.from_mesh(trimesh.Trimesh(
            vertices=vertices, faces=faces), scale=[1, 1, 1])

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

    def show(self):
        if self.mesh is not None:
            self.mesh.show()
        else:
            PCG_ROOT_LOGGER.error(
                'No mesh was generated from the current heightmap')

    def to_sdf(self, mode='visual', sdf_version='1.6',
               filename=None, model_folder=None,
               copy_resources=False):
        from ...utils import PCG_RESOURCES_ROOT_DIR
        sdf = create_sdf_element('heightmap')
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
                'Folder for heightmaps created: {}'.format(folder))

        if copy_resources and self._image_uri is not None:
            PCG_ROOT_LOGGER.info(
                'Copying heightmap image resource <{}> '
                'to model folder <{}>'.format(
                    self._image_uri.absolute_uri, folder))
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

                if self._image_uri.absolute_uri.startswith(
                        os.path.abspath(model_folder)):
                    PCG_ROOT_LOGGER.info(
                        'Image already exists in destination '
                        'folder: {}'.format(model_folder))
                else:
                    if filename is None:
                        image_filename = os.path.basename(
                            self._image_uri.absolute_uri)
                    else:
                        image_filename = filename

                    if not image_filename.endswith('.png'):
                        image_filename += '.png'
                    old_filename = self._image_uri.absolute_uri
                    copyfile(
                        old_filename,
                        os.path.join(folder, image_filename))
                    self._image_uri = Path(
                        os.path.join(folder, image_filename))
                    PCG_ROOT_LOGGER.info(
                        'Heightmap image file was copied from {} to {}'.format(
                            old_filename, self._image_uri.absolute_uri))
        elif self._image_uri is None and self._image is not None:
            self.export(
                filename='heightmap',
                folder=folder,
                format='png')

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
                    sdf.add_texture(texture=texture.to_sdf(
                        sdf_version=sdf_version,
                        model_folder=model_folder,
                        copy_resources=copy_resources
                    ))
        return sdf
