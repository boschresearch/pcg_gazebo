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
from .mechanical_reduction import MechanicalReduction
from .hardware_interface import HardwareInterface


class Actuator(XMLBase):
    _NAME = 'actuator'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        name='actuator'
    )

    _CHILDREN_CREATORS = dict(
        mechanicalReduction=dict(creator=MechanicalReduction),
        hardwareInterface=dict(creator=HardwareInterface)
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
    def mechanicalReduction(self):
        return self._get_child_element('mechanical_reduction')

    @mechanicalReduction.setter
    def mechanicalReduction(self, value):
        self._add_child_element('mechanical_reduction', value)

    @property
    def hardwareInterface(self):
        return self._get_child_element('hardware_interface')

    @hardwareInterface.setter
    def hardwareInterface(self, value):
        self._add_child_element('hardware_interface', value)
