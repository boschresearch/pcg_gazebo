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
from .origin import Origin
from .geometry import Geometry
from .material import Material
from .gazebo import Gazebo


class Visual(XMLBase):
    _NAME = 'visual'
    _TYPE = 'urdf'

    _CHILDREN_CREATORS = dict(
        origin=dict(creator=Origin),
        geometry=dict(creator=Geometry),
        material=dict(creator=Material, optional=True),
        gazebo=dict(creator=Gazebo, optional=True)
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
        assert isinstance(value, str), 'Name input must be a string'
        self.attributes['name'] = value

    @property
    def origin(self):
        return self._get_child_element('origin')

    @origin.setter
    def origin(self, value):
        self._add_child_element('origin', value)

    @property
    def geometry(self):
        return self._get_child_element('geometry')

    @geometry.setter
    def geometry(self, value):
        self._add_child_element('geometry', value)

    @property
    def transparency(self):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo('none', dict(transparency=None))
        return self.children['gazebo']._get_child_element('transparency')

    @transparency.setter
    def transparency(self, value):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo('none', dict(transparency=None))
        self.children['gazebo']._add_child_element('transparency', value)

    @property
    def cast_shadows(self):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo('none', dict(cast_shadows=None))
        return self.children['gazebo']._get_child_element('cast_shadows')

    @cast_shadows.setter
    def cast_shadows(self, value):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo('none', dict(cast_shadows=None))
        self.children['gazebo']._add_child_element('cast_shadows', value)

    @property
    def material(self):
        return self._get_child_element('material')

    @material.setter
    def material(self, value):
        self._add_child_element('material', value)

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('visual')
        obj.name = self.name
        obj.pose = self.origin.to_sdf()
        obj.geometry = self.geometry.to_sdf()
        if 'gazebo' in self.children:
            if 'transparency' in self.children['gazebo'].children:
                obj.transparency = self.transparency
            if 'cast_shadows' in self.children['gazebo'].children:
                obj.cast_shadows = self.cast_shadows
        return obj
