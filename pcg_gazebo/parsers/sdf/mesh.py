# Copyright (c) 2019 - The Procedural Generation for Gazebo authors
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

from ..types import XMLBase
from .submesh import SubMesh
from .uri import URI
from .scale import Scale


class Mesh(XMLBase):
    _NAME = 'mesh'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        uri=dict(creator=URI),
        scale=dict(creator=Scale),
        submesh=dict(creator=SubMesh, n_elems=1, optional=True)
    )

    def __init__(self):
        super(Mesh, self).__init__()
        self.reset()

    @property
    def uri(self):
        return self._get_child_element('uri')

    @uri.setter
    def uri(self, value):
        self._add_child_element('uri', value)

    @property
    def scale(self):
        return self._get_child_element('scale')

    @scale.setter
    def scale(self, value):
        self._add_child_element('scale', value)

    @property
    def submesh(self):
        return self._get_child_element('submesh')

    @submesh.setter
    def submesh(self, values):
        self._add_child_element('submesh', values)

    def is_valid(self):
        if len(self.children) not in [2, 3]:
            print('Mesh must have at least a uri and a scale objects')
            return False
        if 'uri' not in self.children:
            print('Mesh has no item tagged as uri')
            return False
        if 'scale' not in self.children:
            print('Mesh has no item tagged as scale')
            return False
        if not isinstance(self.children['uri'], URI):
            print('Mesh element child is not of type URI')
            return False
        if not isinstance(self.children['scale'], Scale):
            print('Mesh element child is not of type Scale')
            return False
        if 'submesh' in self.children:
            if not isinstance(self.children['submesh'], SubMesh):
                print('Mesh element child is not of type SubMesh')
                return False

        return XMLBase.is_valid(self)

    def random(self):
        self.uri.random()
        self.scale.random()
