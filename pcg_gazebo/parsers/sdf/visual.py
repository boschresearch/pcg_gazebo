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
from .pose import Pose
from .geometry import Geometry
from .material import Material
from .transparency import Transparency
from .cast_shadows import CastShadows


class Visual(XMLBase):
    _NAME = 'visual'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        geometry=dict(creator=Geometry),
        pose=dict(creator=Pose, n_elems=1, optional=True),
        material=dict(creator=Material, n_elems=1, optional=True),
        transparency=dict(
            creator=Transparency, default=[False], n_elems=1, optional=True),
        cast_shadows=dict(
            creator=CastShadows, default=[True], n_elems=1, optional=True)
    )

    _ATTRIBUTES = dict(
        name='visual'
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert isinstance(value, str)
        assert len(value) > 0
        self.attributes['name'] = value

    @property
    def pose(self):
        return self._get_child_element('pose')

    @pose.setter
    def pose(self, value):
        self._add_child_element('pose', value)

    @property
    def transparency(self):
        return self._get_child_element('transparency')

    @transparency.setter
    def transparency(self, value):
        self._add_child_element('transparency', value)

    @property
    def geometry(self):
        return self._get_child_element('geometry')

    @geometry.setter
    def geometry(self, value):
        self._add_child_element('geometry', value)

    @property
    def material(self):
        return self._get_child_element('material')

    @material.setter
    def material(self, value):
        self._add_child_element('material', value)

    @property
    def cast_shadows(self):
        return self._get_child_element('cast_shadows')

    @cast_shadows.setter
    def cast_shadows(self, value):
        self._add_child_element('cast_shadows', value)

    def is_valid(self):
        if len(self.attributes) != 1:
            print('Visual should have at least one attribute')
            return False
        if 'name' not in self.attributes:
            print('Visual should have an attribute <name>')
            return False
        if len(self.attributes['name']) == 0:
            print('Visual name attribute is empty')
            return False

        return XMLBase.is_valid(self)
