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
from ._collection_manager import _CollectionManager
from ..simulation.properties import Mesh
from ..path import Path
from ..log import PCG_ROOT_LOGGER
from ..utils import generate_random_string


class MeshManager(_CollectionManager):
    def __init__(self):
        super(MeshManager, self).__init__()

    @staticmethod
    def get_instance():
        if MeshManager._INSTANCE is None:
            MeshManager._INSTANCE = MeshManager()
        return MeshManager._INSTANCE

    def get_unique_tag(self, length=5):
        label = generate_random_string(length)
        while self.has_element(label):
            label = generate_random_string(length)
        return label

    def add(self, tag, **kwargs):
        if self.has_element(tag):
            return False
        if 'filename' in kwargs:
            if 'scale' not in kwargs:
                kwargs['scale'] = [1, 1, 1]
            self._collection[tag] = Mesh(
                **kwargs)
        elif 'mesh' in kwargs:
            if 'scale' not in kwargs:
                kwargs['scale'] = [1, 1, 1]
            self._collection[tag] = Mesh.from_mesh(
                **kwargs)
        elif 'type' in kwargs:
            if not hasattr(Mesh, 'create_{}'.format(kwargs['type'])):
                PCG_ROOT_LOGGER.error(
                    'Cannot create mesh of type {}'.format(kwargs['type']))
                return False

            if kwargs['type'] == 'box' and 'size' in kwargs:
                self._collection[tag] = Mesh.create_box(size=kwargs['size'])
            elif kwargs['type'] == 'cylinder' and \
                    'radius' in kwargs and \
                    'height' in kwargs:
                self._collection[tag] = Mesh.create_cylinder(
                    radius=kwargs['radius'],
                    height=kwargs['height']
                )
            elif kwargs['type'] == 'capsule' and \
                    'radius' in kwargs and \
                    'height' in kwargs:
                self._collection[tag] = Mesh.create_capsule(
                    radius=kwargs['radius'],
                    height=kwargs['height']
                )
            elif kwargs['type'] == 'sphere' and \
                    'radius' in kwargs:
                self._collection[tag] = Mesh.create_sphere(
                    radius=kwargs['radius'])
            else:
                return False
        else:
            return False
        return True

    def get(self, **kwargs):
        if 'filename' in kwargs:
            mesh_filename = Path(kwargs['filename'])
            if 'scale' in kwargs:
                scale = kwargs['scale']
            else:
                scale = [1, 1, 1]

            for tag in self._collection:
                if self._collection[tag]._uri == mesh_filename and \
                        self._collection[tag]._scale == scale:
                    return self._collection[tag]
        elif 'tag' in kwargs:
            if not self.has_element(kwargs['tag']):
                return None
            return self._collection[kwargs['tag']]
        return None

    def get_tag(self, filename, scale=[1, 1, 1]):
        mesh_filename = Path(filename)
        print('filename: {}'.format(mesh_filename._absolute_uri))
        if mesh_filename is None:
            return None
        for tag in self._collection:
            if self._collection[tag]._uri == mesh_filename and \
                    self._collection[tag]._scale == scale:
                return tag
        return None
