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
from .max_step_size import MaxStepSize
from .real_time_factor import RealTimeFactor
from .real_time_update_rate import RealTimeUpdateRate
from .max_contacts import MaxContacts
from .ode import ODE
from .bullet import Bullet
from .simbody import Simbody


class Physics(XMLBase):
    _NAME = 'physics'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        max_step_size=dict(creator=MaxStepSize, default=[0.001]),
        real_time_factor=dict(creator=RealTimeFactor, default=[1]),
        real_time_update_rate=dict(creator=RealTimeUpdateRate, default=[1000]),
        max_contacts=dict(creator=MaxContacts, default=[20]),
        ode=dict(creator=ODE, mode='ode', optional=True, default=['physics']),
        simbody=dict(creator=Simbody, mode='simbody', optional=True),
        bullet=dict(
            creator=Bullet, mode='bullet', optional=True, default=['physics'])
    )

    _ATTRIBUTES = dict(
        name='default_physics',
        default='1',
        type='ode'
    )

    _MODES = ['ode', 'bullet', 'simbody']

    def __init__(self, mode='ode'):
        XMLBase.__init__(self)
        self.reset(mode=mode)

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert isinstance(value, str)
        assert len(value) > 0
        self.attributes['name'] = value

    @property
    def default(self):
        return self.attributes['default']

    @default.setter
    def default(self, value):
        assert isinstance(value, bool) or value in [0, 1], \
            'Physics default attribute must be a' \
            ' boolean, provided={}'.format(value)
        self.attributes['default'] = '1' if value else '0'

    @property
    def type(self):
        return self.attributes['type']

    @type.setter
    def type(self, value):
        assert isinstance(value, str)
        assert value in ['ode', 'bullet', 'dart', 'simbody']
        self.attributes['type'] = value

    @property
    def max_step_size(self):
        return self._get_child_element('max_step_size')

    @max_step_size.setter
    def max_step_size(self, value):
        self._add_child_element('max_step_size', value)

    @property
    def real_time_factor(self):
        return self._get_child_element('real_time_factor')

    @real_time_factor.setter
    def real_time_factor(self, value):
        self._add_child_element('real_time_factor', value)

    @property
    def real_time_update_rate(self):
        return self._get_child_element('real_time_update_rate')

    @real_time_update_rate.setter
    def real_time_update_rate(self, value):
        self._add_child_element('real_time_update_rate', value)

    @property
    def max_contacts(self):
        return self._get_child_element('max_contacts')

    @max_contacts.setter
    def max_contacts(self, value):
        self._add_child_element('max_contacts', value)

    @property
    def ode(self):
        return self._get_child_element('ode')

    @ode.setter
    def ode(self, value):
        self._add_child_element('ode', value)

    @property
    def bullet(self):
        return self._get_child_element('bullet')

    @bullet.setter
    def bullet(self, value):
        self._add_child_element('bullet', value)

    @property
    def simbody(self):
        return self._get_child_element('simbody')

    @simbody.setter
    def simbody(self, value):
        self._add_child_element('simbody', value)

    def reset(self, mode=None, with_optional_elements=False):
        XMLBase.reset(self, mode, with_optional_elements)
        if mode is not None:
            assert isinstance(mode, str)
            assert mode in ['ode', 'bullet', 'dart', 'simbody']
            self.attributes['type'] = mode

    def is_valid(self):
        if len(self.attributes) != 3:
            print('Physics should have three attributes')
            return False
        if 'name' not in self.attributes:
            print('Physics should have an attribute <name>')
            return False
        if len(self.attributes['name']) == 0:
            print('Physics name attribute is empty')
            return False
        if 'default' not in self.attributes:
            print('Physics should have an attribute <default>')
            return False
        if 'type' not in self.attributes:
            print('Physics should have an attribute <type>')
            return False
        if self.attributes['type'] not in ['ode', 'bullet', 'simbody', 'dart']:
            print(
                'Physics type attribute should be ode,'
                ' bullet, simbody or dart')
            return False
        if len(self.children) < 4:
            print('Physics should have at least 4 child elements')
            return False
        return XMLBase.is_valid(self)
