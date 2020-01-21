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
from .gazebo import Gazebo


class Dynamics(XMLBase):
    _NAME = 'dynamics'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        damping='0',
        friction='0'
    )

    _CHILDREN_CREATORS = dict(
        gazebo=dict(creator=Gazebo,
                    optional=True,
                    default=['none', dict(
                        spring_reference=None,
                        spring_stiffness=None)])
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def damping(self):
        return float(self.attributes['damping'])

    @damping.setter
    def damping(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        self.attributes['damping'] = '{}'.format(value)

    @property
    def friction(self):
        return float(self.attributes['friction'])

    @friction.setter
    def friction(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        self.attributes['friction'] = '{}'.format(value)

    @property
    def spring_reference(self):
        if 'gazebo' in self.children:
            return self.children['gazebo']._get_child_element(
                'spring_reference')
        else:
            return None

    @spring_reference.setter
    def spring_reference(self, value):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo(
                *self._CHILDREN_CREATORS['gazebo']['default'])
        self.children['gazebo'].children['spring_reference'].value = value

    @property
    def spring_stiffness(self):
        if 'gazebo' in self.children:
            return self.children['gazebo']._get_child_element(
                'spring_stiffness')
        else:
            return None

    @spring_stiffness.setter
    def spring_stiffness(self, value):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo(
                *self._CHILDREN_CREATORS['gazebo']['default'])
        self.children['gazebo'].children['spring_stiffness'].value = value

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('dynamics')
        obj.damping = self.damping
        obj.friction = self.friction
        if self.spring_reference is not None:
            obj.spring_reference = self.spring_reference
        if self.spring_stiffness is not None:
            obj.spring_stiffness = self.spring_stiffness
        return obj
