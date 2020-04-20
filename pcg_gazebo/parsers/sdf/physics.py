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
from .provide_feedback import ProvideFeedback


class Physics(XMLBase):
    _NAME = 'physics'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        max_step_size=dict(
            creator=MaxStepSize,
            mode=['ode', 'bullet', 'simbody'],
            default=[0.001]),
        real_time_factor=dict(
            creator=RealTimeFactor,
            mode=['ode', 'bullet', 'simbody'],
            default=[1]),
        real_time_update_rate=dict(
            creator=RealTimeUpdateRate,
            mode=['ode', 'bullet', 'simbody'],
            default=[1000]),
        max_contacts=dict(
            creator=MaxContacts,
            mode=['ode', 'bullet', 'simbody'],
            default=[20],
            optional=True),
        ode=dict(
            creator=ODE, mode=['joint', 'ode'],
            optional=True,
            default=['physics']),
        simbody=dict(
            creator=Simbody,
            default=['physics'],
            mode=['joint', 'simbody'],
            optional=True),
        bullet=dict(
            creator=Bullet,
            mode='bullet',
            optional=True,
            default=['physics']),
        provide_feedback=dict(
            creator=ProvideFeedback,
            default=[False],
            mode='joint',
            optional=True)
    )

    _ATTRIBUTES = dict(
        name='default_physics',
        default='1',
        type='ode'
    )

    _ATTRIBUTES_MODES = dict(
        name=['ode', 'bullet', 'simbody'],
        default=['ode', 'bullet', 'simbody'],
        type=['ode', 'bullet', 'simbody']
    )

    _MODES = ['ode', 'bullet', 'simbody', 'joint']

    def __init__(self, mode='ode'):
        super(Physics, self).__init__()
        if mode == 'joint':
            self._CHILDREN_CREATORS['simbody']['default'] = ['joint']
            self._CHILDREN_CREATORS['ode']['default'] = ['joint']
        else:
            self._CHILDREN_CREATORS['simbody']['default'] = ['physics']
            self._CHILDREN_CREATORS['ode']['default'] = ['physics']
        self.reset(mode=mode)

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        if self._mode == 'joint':
            self._mode = 'ode'
        assert self._is_string(value)
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
        if self._mode != value:
            self._mode = value
        self.attributes['type'] = value

    @property
    def max_step_size(self):
        return self._get_child_element('max_step_size')

    @max_step_size.setter
    def max_step_size(self, value):
        if self._mode not in ['ode', 'bullet', 'simbody']:
            self._mode = 'ode'
        self._add_child_element('max_step_size', value)

    @property
    def real_time_factor(self):
        return self._get_child_element('real_time_factor')

    @real_time_factor.setter
    def real_time_factor(self, value):
        if self._mode not in ['ode', 'bullet', 'simbody']:
            self._mode = 'ode'
        self._add_child_element('real_time_factor', value)

    @property
    def real_time_update_rate(self):
        return self._get_child_element('real_time_update_rate')

    @real_time_update_rate.setter
    def real_time_update_rate(self, value):
        if self._mode not in ['ode', 'bullet', 'simbody']:
            self._mode = 'ode'
        self._add_child_element('real_time_update_rate', value)

    @property
    def max_contacts(self):
        return self._get_child_element('max_contacts')

    @max_contacts.setter
    def max_contacts(self, value):
        if self._mode not in ['ode', 'bullet', 'simbody']:
            self._mode = 'ode'
        self._add_child_element('max_contacts', value)

    @property
    def ode(self):
        return self._get_child_element('ode')

    @ode.setter
    def ode(self, value):
        if self._mode != 'ode':
            self._mode = 'ode'
        self._add_child_element('ode', value)

    @property
    def bullet(self):
        return self._get_child_element('bullet')

    @bullet.setter
    def bullet(self, value):
        if self._mode != 'bullet':
            self._mode = 'bullet'
        self._add_child_element('bullet', value)

    @property
    def simbody(self):
        return self._get_child_element('simbody')

    @simbody.setter
    def simbody(self, value):
        if self._mode != 'simbody':
            self._mode = 'simbody'
        self._add_child_element('simbody', value)

    @property
    def provide_feedback(self):
        return self._get_child_element('provide_feedback')

    @provide_feedback.setter
    def provide_feedback(self, value):
        if self._mode != 'joint':
            self._mode = 'joint'
        self._add_child_element('provide_feedback', value)

    def reset(self, mode=None, with_optional_elements=False):
        if mode is not None:
            if mode == 'joint':
                self._CHILDREN_CREATORS['simbody']['default'] = ['joint']
                self._CHILDREN_CREATORS['ode']['default'] = ['joint']
            else:
                self._CHILDREN_CREATORS['simbody']['default'] = ['physics']
                self._CHILDREN_CREATORS['ode']['default'] = ['physics']
        XMLBase.reset(self, mode, with_optional_elements)
        if mode is not None:
            assert isinstance(mode, str)
            if mode != 'joint':
                assert mode in ['ode', 'bullet', 'dart', 'simbody']
                self.attributes['type'] = mode
