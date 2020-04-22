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
from .cfm import CFM
from .erp import ERP
from .contact_max_correcting_vel import ContactMaxCorrectingVel
from .contact_surface_layer import ContactSurfaceLayer
from .split_impulse import SplitImpulse
from .split_impulse_penetration_threshold import \
    SplitImpulsePenetrationThreshold


class Constraints(XMLBase):
    _NAME = 'constraints'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        cfm=dict(creator=CFM),
        erp=dict(creator=ERP),
        contact_surface_layer=dict(creator=ContactSurfaceLayer),
        contact_max_correcting_vel=dict(
            creator=ContactMaxCorrectingVel, mode='ode'),
        split_impulse=dict(
            creator=SplitImpulse, mode='bullet'),
        split_impulse_penetration_threshold=dict(
            creator=SplitImpulsePenetrationThreshold, mode='bullet'))

    _MODES = ['ode', 'bullet']

    def __init__(self, engine='ode'):
        super(Constraints, self).__init__()
        self.reset(mode=engine)

    @property
    def cfm(self):
        return self._get_child_element('cfm')

    @cfm.setter
    def cfm(self, value):
        self._add_child_element('cfm', value)

    @property
    def erp(self):
        return self._get_child_element('erp')

    @erp.setter
    def erp(self, value):
        self._add_child_element('erp', value)

    @property
    def contact_surface_layer(self):
        return self._get_child_element('contact_surface_layer')

    @contact_surface_layer.setter
    def contact_surface_layer(self, value):
        self._add_child_element('contact_surface_layer', value)

    @property
    def contact_max_correcting_vel(self):
        return self._get_child_element('contact_max_correcting_vel')

    @contact_max_correcting_vel.setter
    def contact_max_correcting_vel(self, value):
        self._add_child_element('contact_max_correcting_vel', value)

    @property
    def split_impulse(self):
        return self._get_child_element('split_impulse')

    @split_impulse.setter
    def split_impulse(self, value):
        self._add_child_element('split_impulse', value)

    @property
    def split_impulse_penetration_threshold(self):
        return self._get_child_element('split_impulse_penetration_threshold')

    @split_impulse_penetration_threshold.setter
    def split_impulse_penetration_threshold(self, value):
        self._add_child_element('split_impulse_penetration_threshold', value)

    def is_valid(self):
        if self._mode == 'ode':
            if len(self.children) != 4:
                print('Constraints must have 4 child objects')
                return False
            if 'contact_max_correcting_vel' not in self.children:
                print(
                    'Constraints has no item tagged'
                    ' as contact_max_correcting_vel')
                return False
            if not isinstance(
                    self.children['contact_max_correcting_vel'],
                    ContactMaxCorrectingVel):
                print(
                    'Constraints element child is not'
                    ' of type contact_max_correcting_vel')
                return False
        if self._mode == 'bullet':
            if len(self.children) != 5:
                print('Constraints must have 5 child objects')
                return False
            if 'split_impulse' not in self.children:
                print('Constraints has no item tagged as split_impulse')
                return False
            if not isinstance(self.children['split_impulse'], SplitImpulse):
                print('Constraints element child is not of type split_impulse')
                return False
            if 'split_impulse_penetration_threshold' not in self.children:
                print(
                    'Constraints has no item tagged'
                    ' as split_impulse_penetration_threshold')
                return False
            if not isinstance(
                    self.children['split_impulse_penetration_threshold'],
                    SplitImpulsePenetrationThreshold):
                print(
                    'Constraints element child is not'
                    ' of type split_impulse_penetration_threshold')
                return False
        if 'cfm' not in self.children:
            print('Constraints has no item tagged as CFM')
            return False
        if 'erp' not in self.children:
            print('Constraints has no item tagged as ERP')
            return False
        if 'contact_surface_layer' not in self.children:
            print('Constraints has no item tagged as contact_surface_layer')
            return False
        if not isinstance(self.children['cfm'], CFM):
            print('Constraints element child is not of type CFM')
            return False
        if not isinstance(self.children['erp'], ERP):
            print('Constraints element child is not of type ERP')
            return False
        if not isinstance(
                self.children['contact_surface_layer'],
                ContactSurfaceLayer):
            print(
                'Constraints element child is'
                ' not of type ContactSurfaceLayer')
            return False
        return XMLBase.is_valid(self)
