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
from .lower import Lower
from .upper import Upper
from .effort import Effort
from .velocity import Velocity
from .stiffness import Stiffness
from .dissipation import Dissipation
from .cfm import CFM
from .erp import ERP


class Limit(XMLBase):
    _NAME = 'limit'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        lower=dict(creator=Lower, default=[-1e16], mode='axis'),
        upper=dict(creator=Upper, default=[1e16], mode='axis'),
        effort=dict(creator=Effort, default=[-1], optional=True, mode='axis'),
        velocity=dict(
            creator=Velocity, default=[-1.0], optional=True, mode='axis'),
        stiffness=dict(
            creator=Stiffness, default=[1e8], optional=True, mode='axis'),
        dissipation=dict(
            creator=Dissipation, default=[1], optional=True, mode='axis'),
        cfm=dict(creator=CFM, default=[0], optional=True, mode='joint'),
        erp=dict(creator=ERP, default=[0], optional=True, mode='joint')
    )

    _MODES = ['axis', 'joint']

    def __init__(self, mode='axis'):
        super(Limit, self).__init__()
        self.reset(mode=mode)

    @property
    def lower(self):
        return self._get_child_element('lower')

    @lower.setter
    def lower(self, value):
        if self._mode != 'axis':
            self.reset('axis')
        self._add_child_element('lower', value)

    @property
    def upper(self):
        return self._get_child_element('upper')

    @upper.setter
    def upper(self, value):
        if self._mode != 'axis':
            self.reset('axis')
        self._add_child_element('upper', value)

    @property
    def effort(self):
        return self._get_child_element('effort')

    @effort.setter
    def effort(self, value):
        if self._mode != 'axis':
            self.reset('axis')
        self._add_child_element('effort', value)

    @property
    def velocity(self):
        return self._get_child_element('velocity')

    @velocity.setter
    def velocity(self, value):
        if self._mode != 'axis':
            self.reset('axis')
        self._add_child_element('velocity', value)

    @property
    def stiffness(self):
        return self._get_child_element('stiffness')

    @stiffness.setter
    def stiffness(self, value):
        if self._mode != 'axis':
            self.reset('axis')
        self._add_child_element('stiffness', value)

    @property
    def dissipation(self):
        return self._get_child_element('dissipation')

    @dissipation.setter
    def dissipation(self, value):
        if self._mode != 'axis':
            self.reset('axis')
        self._add_child_element('dissipation', value)

    @property
    def cfm(self):
        return self._get_child_element('cfm')

    @cfm.setter
    def cfm(self, value):
        if self._mode != 'joint':
            self.reset('joint')
        self._add_child_element('cfm', value)

    @property
    def erp(self):
        return self._get_child_element('erp')

    @erp.setter
    def erp(self, value):
        if self._mode != 'joint':
            self.reset('joint')
        self._add_child_element('erp', value)
