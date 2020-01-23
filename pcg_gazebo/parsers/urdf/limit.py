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
import random
from .gazebo import Gazebo
from ..types import XMLBase
from ...utils import is_scalar


class Limit(XMLBase):
    _NAME = 'limit'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        lower='0',
        upper='0',
        effort='0',
        velocity='0'
    )

    _CHILDREN_CREATORS = dict(
        gazebo=dict(creator=Gazebo,
                    optional=True,
                    default=['none', dict(
                        stiffness=None,
                        dissipation=None)])
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def lower(self):
        return float(self.attributes['lower'])

    @lower.setter
    def lower(self, value):
        assert is_scalar(value), 'Lower value must be a scalar'
        self.attributes['lower'] = '{}'.format(value)

    @property
    def upper(self):
        return float(self.attributes['upper'])

    @upper.setter
    def upper(self, value):
        assert is_scalar(value), 'Upper value must be a scalar'
        self.attributes['upper'] = '{}'.format(value)

    @property
    def effort(self):
        return float(self.attributes['effort'])

    @effort.setter
    def effort(self, value):
        assert is_scalar(value), 'Effort value must be a scalar'
        self.attributes['effort'] = '{}'.format(value)

    @property
    def velocity(self):
        return float(self.attributes['velocity'])

    @velocity.setter
    def velocity(self, value):
        assert is_scalar(value), 'Velocity value must be a scalar'
        self.attributes['velocity'] = '{}'.format(value)

    @property
    def stiffness(self):
        if 'gazebo' in self.children:
            return self.children['gazebo']._get_child_element('stiffness')
        else:
            return None

    @stiffness.setter
    def stiffness(self, value):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo(
                *self._CHILDREN_CREATORS['gazebo']['default'])
        self.children['gazebo'].children['stiffness'].value = value

    @property
    def dissipation(self):
        if 'gazebo' in self.children:
            return self.children['gazebo']._get_child_element('dissipation')
        else:
            return None

    @dissipation.setter
    def dissipation(self, value):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo(
                *self._CHILDREN_CREATORS['gazebo']['default'])
        self.children['gazebo'].children['dissipation'].value = value

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('limit')
        obj.upper = self.upper
        obj.lower = self.lower
        obj.effort = self.effort
        obj.velocity = self.velocity
        if self.stiffness is not None:
            obj.stiffness = self.stiffness
        if self.dissipation is not None:
            obj.dissipation = self.dissipation
        return obj

    def random(self):
        self.lower = random.random()
        self.upper = self.lower + 1
        self.velocity = random.random()
        self.effort = random.random()
