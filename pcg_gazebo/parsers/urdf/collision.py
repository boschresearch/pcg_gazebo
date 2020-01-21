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
from .gazebo import Gazebo


class Collision(XMLBase):
    _NAME = 'collision'
    _TYPE = 'urdf'

    _CHILDREN_CREATORS = dict(
        origin=dict(creator=Origin),
        geometry=dict(creator=Geometry),
        gazebo=dict(
            creator=Gazebo,
            optional=True,
            default=['none', dict(max_contacts=None, surface=None)])
    )

    _ATTRIBUTES = dict(
        name='collision'
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
    def max_contacts(self):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo('none', dict(max_contacts=None))
        return self.children['gazebo']._get_child_element('max_contacts')

    @max_contacts.setter
    def max_contacts(self, value):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo('none', dict(max_contacts=None))
        self.children['gazebo']._add_child_element('max_contacts', value)

    @property
    def surface(self):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo('none', dict(surface=None))
        return self.children['gazebo']._get_child_element('surface')

    @surface.setter
    def surface(self, value):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo('none', dict(surface=None))
        self.children['gazebo']._add_child_element('surface', value)

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('collision')
        obj.name = self.name
        obj.pose = self.origin.to_sdf()
        obj.geometry = self.geometry.to_sdf()
        if 'gazebo' in self.children:
            if 'max_contacts' in self.children['gazebo'].children:
                obj.max_contacts = self.max_contacts
            if 'surface' in self.children['gazebo'].children:
                obj.surface = self.surface

        return obj
