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
from ._collection_manager import _CollectionManager
from ..path import Path
from ..log import PCG_ROOT_LOGGER
from ..utils import is_array, is_scalar
import trimesh


class MeshManager(_CollectionManager):
    def __init__(self):
        super(MeshManager, self).__init__()

    @staticmethod
    def get_instance():
        if MeshManager._INSTANCE is None:
            MeshManager._INSTANCE = MeshManager()
        return MeshManager._INSTANCE

    def get_unique_tag(self, length=5):
        i = 0
        label = 'mesh_{}'.format(i)
        while self.has_element(label):
            i += 1
            label = 'mesh_{}'.format(i)
        return label

    def add(self, tag=None, **kwargs):
        if tag is None:
            tag = self.get_unique_tag()
        if self.has_element(tag):
            return None
        self._collection[tag] = dict(filename=None)
        if 'filename' in kwargs:
            assert os.path.isfile(kwargs['filename']), \
                'Invalid mesh filename, value={}'.format(kwargs['filename'])
            cur_tag = self.find_by_filename(kwargs['filename'])
            if cur_tag is not None:
                return cur_tag
            self._collection[tag]['filename'] = Path(kwargs['filename'])
            self._collection[tag]['mesh'] = trimesh.load_mesh(
                kwargs['filename'])
            if isinstance(self._collection[tag]['mesh'], trimesh.Scene):
                meshes = list(self._collection[tag]['mesh'].dump())
                PCG_ROOT_LOGGER.info('# meshes={}, filename={}'.format(
                    len(meshes), self._collection[tag]['filename']))
                if len(meshes) == 1:
                    self._collection[tag]['mesh'] = meshes[0]
        elif 'mesh' in kwargs:
            self._collection[tag]['mesh'] = kwargs['mesh']
            if isinstance(self._collection[tag]['mesh'], trimesh.Scene):
                meshes = list(self._collection[tag]['mesh'].dump())
                PCG_ROOT_LOGGER.info('# meshes={}'.format(
                    len(meshes)))
                if len(meshes) == 1:
                    self._collection[tag]['mesh'] = meshes[0]
        elif 'type' in kwargs:
            mesh_tag = self.find_by_parameters(**kwargs)
            if mesh_tag is not None:
                print(
                    'find_by_parameters=', mesh_tag, kwargs,
                    self._collection[mesh_tag])
                tag = mesh_tag
                return tag
            if kwargs['type'] not in ['box', 'cylinder', 'capsule', 'sphere']:
                return None
            else:
                if kwargs['type'] == 'box' and 'size' in kwargs:
                    assert is_array(kwargs['size']), \
                        'Size is not an array'
                    vec = list(kwargs['size'])
                    assert len(vec) == 3, \
                        'Input size array must have 3 elements'
                    for elem in vec:
                        assert is_scalar(elem), \
                            'Vector element must be a scalar'
                        assert elem > 0, \
                            'Size vector components must be greater than zero'
                elif kwargs['type'] == 'cylinder' and \
                        'radius' in kwargs and \
                        'height' in kwargs:
                    assert is_scalar(kwargs['radius']), \
                        'Radius must be a scalar'
                    assert kwargs['radius'] > 0, \
                        'Cylinder radius must be greater than zero'
                    assert is_scalar(kwargs['height']), \
                        'Height must be a scalar'
                    assert kwargs['height'] > 0, \
                        'Cylinder height must be greater than zero'
                elif kwargs['type'] == 'capsule' and \
                        'radius' in kwargs and \
                        'height' in kwargs:
                    assert is_scalar(kwargs['radius']), \
                        'Radius must be a scalar'
                    assert kwargs['radius'] > 0, \
                        'Capsule radius must be greater than zero'
                    assert is_scalar(kwargs['height']), \
                        'Height must be a scalar'
                    assert kwargs['height'] > 0, \
                        'Capsule height must be greater than zero'
                elif kwargs['type'] == 'sphere' and \
                        'radius' in kwargs:
                    assert is_scalar(kwargs['radius']), \
                        'Radius must be a scalar'
                    assert kwargs['radius'] > 0, \
                        'Sphere radius must be greater than zero'
                self._collection[tag].update(kwargs)
        else:
            del self._collection[tag]
            return None
        return tag

    def get(self, **kwargs):
        if 'filename' in kwargs:
            mesh_filename = Path(kwargs['filename'])

            for tag in self._collection:
                if self._collection[tag]['filename']._uri == mesh_filename:
                    return self._collection[tag]['mesh']
        elif 'tag' in kwargs:
            if not self.has_element(kwargs['tag']):
                PCG_ROOT_LOGGER.error(
                    'No element with tag <{}> was found'.format(
                        kwargs['tag']))
                return None
            if 'type' in self._collection[kwargs['tag']]:
                if self._collection[kwargs['tag']]['type'] == 'box':
                    return trimesh.creation.box(
                        extents=self._collection[kwargs['tag']]['size'])
                elif self._collection[kwargs['tag']]['type'] == 'cylinder':
                    return trimesh.creation.cylinder(
                        radius=self._collection[kwargs['tag']]['radius'],
                        height=self._collection[kwargs['tag']]['height'])
                elif self._collection[kwargs['tag']]['type'] == 'capsule':
                    return trimesh.creation.capsule(
                        radius=self._collection[kwargs['tag']]['radius'],
                        height=self._collection[kwargs['tag']]['height'])
                elif self._collection[kwargs['tag']]['type'] == 'sphere':
                    return trimesh.creation.icosphere(
                        radius=self._collection[kwargs['tag']]['radius'])
            else:
                return self._collection[kwargs['tag']]['mesh']
        elif 'type' in kwargs:
            if kwargs['type'] == 'box':
                assert 'size' in kwargs, \
                    'Box size not provided'
                assert is_array(kwargs['size']), \
                    'Size is not an array'
                vec = list(kwargs['size'])
                assert len(vec) == 3, \
                    'Input size array must have 3 elements'
                for elem in vec:
                    assert is_scalar(elem), \
                        'Vector element must be a scalar'
                    assert elem > 0, \
                        'Size vector components must be greater than zero'
                return trimesh.creation.box(
                    extents=kwargs['size'])
            elif kwargs['type'] == 'cylinder':
                assert 'radius' in kwargs and 'height' in kwargs, \
                    'Radius and height not provided'
                assert is_scalar(kwargs['radius']), \
                    'Radius must be a scalar'
                assert kwargs['radius'] > 0, \
                    'Cylinder radius must be greater than zero'
                assert is_scalar(kwargs['height']), \
                    'Height must be a scalar'
                assert kwargs['height'] > 0, \
                    'Cylinder height must be greater than zero'
                return trimesh.creation.cylinder(
                    radius=kwargs['radius'],
                    height=kwargs['height'])
            elif kwargs['type'] == 'capsule':
                assert 'radius' in kwargs and 'height' in kwargs, \
                    'Radius and height not provided'
                assert is_scalar(kwargs['radius']), \
                    'Radius must be a scalar'
                assert kwargs['radius'] > 0, \
                    'Capsule radius must be greater than zero'
                assert is_scalar(kwargs['height']), \
                    'Height must be a scalar'
                assert kwargs['height'] > 0, \
                    'Capsule height must be greater than zero'
                return trimesh.creation.capsule(
                    radius=kwargs['radius'],
                    height=kwargs['height'])
            elif kwargs['type'] == 'sphere':
                assert 'radius' in kwargs, 'No radius provided'
                assert is_scalar(kwargs['radius']), \
                    'Radius must be a scalar'
                assert kwargs['radius'] > 0, \
                    'Sphere radius must be greater than zero'
                return trimesh.creation.icosphere(
                    radius=kwargs['radius'])

        return None

    def find_by_filename(self, filename):
        mesh_filename = Path(filename)
        if mesh_filename is None:
            return None
        for tag in self._collection:
            if self._collection[tag]['filename'] is not None:
                if self._collection[tag]['filename'] == mesh_filename:
                    return tag
        return None

    def find_by_parameters(self, **kwargs):
        if 'type' not in kwargs:
            return None
        for tag in self._collection:
            found_mesh = False
            for param_tag in kwargs:
                if param_tag not in self._collection[tag]:
                    found_mesh = False
                    break
                if (is_array(self._collection[tag][param_tag]) and
                    not is_array(kwargs[param_tag])) or \
                        (not is_array(self._collection[tag][param_tag]) and
                         is_array(kwargs[param_tag])):
                    found_mesh = False
                    break
                if is_array(self._collection[tag][param_tag]) and \
                        is_array(kwargs[param_tag]):
                    if len(self._collection[tag][param_tag]) != \
                            len(kwargs[param_tag]):
                        found_mesh = False
                        break
                    if np.sum(np.array(self._collection[tag][param_tag]) -
                              np.array(kwargs[param_tag])) != 0:
                        found_mesh = False
                        break
                elif self._collection[tag][param_tag] != kwargs[param_tag]:
                    found_mesh = False
                    break
                found_mesh = True
            if found_mesh:
                return tag
        return None
