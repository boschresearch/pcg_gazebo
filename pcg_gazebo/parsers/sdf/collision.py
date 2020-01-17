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
from .max_contacts import MaxContacts
from .surface import Surface
from .laser_retro import LaserRetro


class Collision(XMLBase):
    _NAME = 'collision'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        geometry=dict(creator=Geometry),
        pose=dict(creator=Pose, n_elems=1, optional=True),
        max_contacts=dict(
            creator=MaxContacts, n_elems=1, default=[10], optional=True),
        surface=dict(creator=Surface, optional=True),
        laser_retro=dict(creator=LaserRetro, default=[0], optional=True)
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
        assert isinstance(value, str), 'Collision name must be a string'
        assert len(value) > 0, 'Collision name string is empty'
        self.attributes['name'] = value

    @property
    def pose(self):
        return self._get_child_element('pose')

    @pose.setter
    def pose(self, vec):
        self._add_child_element('pose', vec)

    @property
    def geometry(self):
        return self._get_child_element('geometry')

    @geometry.setter
    def geometry(self, value):
        self._add_child_element('geometry', value)

    @property
    def max_contacts(self):
        return self._get_child_element('max_contacts')

    @max_contacts.setter
    def max_contacts(self, value):
        self._add_child_element('max_contacts', value)

    @property
    def surface(self):
        return self._get_child_element('surface')

    @surface.setter
    def surface(self, value):
        self._add_child_element('surface', value)

    @property
    def laser_retro(self):
        return self._get_child_element('laser_retro')

    @laser_retro.setter
    def laser_retro(self, value):
        self._add_child_element('laser_retro', value)

    def is_valid(self):
        if len(self.attributes) != 1:
            print('Collision should have at least one attribute')
            return False
        if 'name' not in self.attributes:
            print('Collision should have an attribute <name>')
            return False
        if len(self.attributes['name']) == 0:
            print('Collision name attribute is empty')
            return False
        return XMLBase.is_valid(self)
