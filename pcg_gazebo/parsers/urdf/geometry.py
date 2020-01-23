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
from .box import Box
from .cylinder import Cylinder
from .sphere import Sphere
from .mesh import Mesh


class Geometry(XMLBase):
    _NAME = 'geometry'
    _TYPE = 'urdf'

    _CHILDREN_CREATORS = dict(
        box=dict(creator=Box, mode='box', n_elems=1),
        cylinder=dict(creator=Cylinder, mode='cylinder', n_elems=1),
        sphere=dict(creator=Sphere, mode='sphere', n_elems=1),
        mesh=dict(creator=Mesh, mode='mesh', n_elems=1)
    )
    _MODES = ['box', 'cylinder', 'sphere', 'mesh']

    def __init__(self):
        XMLBase.__init__(self)
        self.reset('box')

    @property
    def box(self):
        return self._get_child_element('box')

    @box.setter
    def box(self, value):
        self.reset(mode='box')
        self._add_child_element('box', value)

    @property
    def cylinder(self):
        return self._get_child_element('cylinder')

    @cylinder.setter
    def cylinder(self, value):
        self.reset(mode='cylinder')
        self._add_child_element('cylinder', value)

    @property
    def sphere(self):
        return self._get_child_element('sphere')

    @sphere.setter
    def sphere(self, value):
        self.reset(mode='sphere')
        self._add_child_element('sphere', value)

    @property
    def mesh(self):
        return self._get_child_element('mesh')

    @mesh.setter
    def mesh(self, value):
        self.reset(mode='mesh')
        self._add_child_element('mesh', value)

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('geometry')
        obj.reset(self._mode)
        if self._get_child_element('box') is not None:
            obj.box = self.box.to_sdf()
        elif self._get_child_element('sphere') is not None:
            obj.sphere = self.sphere.to_sdf()
        elif self._get_child_element('cylinder') is not None:
            obj.cylinder = self.cylinder.to_sdf()
        elif self._get_child_element('mesh') is not None:
            obj.mesh = self.mesh.to_sdf()
        return obj

    def random(self):
        for tag in self._MODES:
            if self._get_child_element(tag) is not None:
                getattr(self, tag).random()
