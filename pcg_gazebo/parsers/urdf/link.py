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
from ...utils import is_string
from .inertial import Inertial
from .collision import Collision
from .gazebo import Gazebo
from .visual import Visual


class Link(XMLBase):
    _NAME = 'link'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        name='link'
    )

    _CHILDREN_CREATORS = dict(
        inertial=dict(creator=Inertial, n_elems=1, optional=True),
        collision=dict(creator=Collision, n_elems='+', optional=True),
        visual=dict(creator=Visual, n_elems='+', optional=True),
        gazebo=dict(creator=Gazebo, optional=True, default=['link'])
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert is_string(value), \
            'Link name should be string or unicode'
        assert len(value) > 0, 'Name string cannot be empty'
        self.attributes['name'] = str(value)

    @property
    def mass(self):
        if 'inertial' in self.children:
            return self.children['inertial'].mass
        else:
            return None

    @mass.setter
    def mass(self, value):
        if 'inertial' not in self.children:
            self.children['inertial'] = Inertial()
        self.children['inertial'].mass = value

    @property
    def center_of_mass(self):
        if 'inertial' in self.children:
            return self.children['inertial'].origin.xyz
        else:
            return None

    @center_of_mass.setter
    def center_of_mass(self, vec):
        if 'inertial' not in self.children:
            self.children['inertial'] = Inertial()
        assert isinstance(vec, list)
        assert len(vec) == 3
        for elem in vec:
            assert isinstance(elem, float) or isinstance(elem, int)
        self.children['inertial'].origin.xyz = vec

    @property
    def inertia(self):
        if 'inertial' in self.children:
            return self.children['inertial'].inertia
        else:
            return None

    @inertia.setter
    def inertia(self, values):
        if 'inertial' not in self.children:
            self.children['inertial'] = Inertial()
        self.children['inertial'].inertia = values

    @property
    def inertial(self):
        return self._get_child_element('inertial')

    @inertial.setter
    def inertial(self, value):
        self._add_child_element('inertial', value)

    @property
    def collisions(self):
        return self._get_child_element('collision')

    @property
    def visuals(self):
        return self._get_child_element('visual')

    @property
    def gazebo(self):
        return self._get_child_element('gazebo')

    @gazebo.setter
    def gazebo(self, value):
        self._add_child_element('gazebo', value)

    def add_collision(self, name, collision=None):
        if self.collisions is not None:
            for elem in self.collisions:
                if elem.name == name:
                    print(
                        'Collision element with name <{}>'
                        ' already exists'.format(name))
                    return
        if collision is not None:
            self._add_child_element('collision', collision)
        else:
            collision = Collision()
            self._add_child_element('collision', collision)
        self.children['collision'][-1].name = name

    def get_collision_by_name(self, name):
        if self.collisions is None:
            return None
        else:
            for elem in self.collisions:
                if elem.name == name:
                    return elem
        return None

    def add_visual(self, name, visual=None):
        if self.visuals is not None:
            for elem in self.visuals:
                if elem.name == name:
                    print(
                        'Visual element with name <{}>'
                        ' already exists'.format(name))
                    return
        if visual is not None:
            self._add_child_element('visual', visual)
        else:
            visual = Visual()
            self._add_child_element('visual', visual)
        self.children['visual'][-1].name = name

    def get_visual_by_name(self, name):
        if self.visuals is None:
            return None
        else:
            for elem in self.visuals:
                if elem.name == name:
                    return elem
        return None
