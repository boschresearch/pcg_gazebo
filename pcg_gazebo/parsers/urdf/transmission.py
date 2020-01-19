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
from .type import Type
from .actuator import Actuator
from .hardware_interface import HardwareInterface


class JointTransmission(XMLBase):
    _NAME = 'joint'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        name='joint'
    )

    _CHILDREN_CREATORS = dict(
        hardwareInterface=dict(
            creator=HardwareInterface,
            default=['EffortJointInterface']))

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
    def hardwareInterface(self):
        return self._get_child_element('hardwareInterface')

    @hardwareInterface.setter
    def hardwareInterface(self, value):
        self._add_child_element('hardwareInterface')


class Transmission(XMLBase):
    _NAME = 'transmission'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        name='transmission'
    )

    _CHILDREN_CREATORS = dict(
        type=dict(
            creator=Type,
            default=['transmission_interface/SimpleTransmission']),
        actuator=dict(
            creator=Actuator),
        joint=dict(
            creator=JointTransmission))

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
    def type(self):
        return self._get_child_element('type')

    @type.setter
    def type(self, value):
        self._add_child_element('type', value)

    @property
    def actuator(self):
        return self._get_child_element('actuator')

    @actuator.setter
    def actuator(self, value):
        self._add_child_element('actuator', value)

    @property
    def joint(self):
        return self._get_child_element('joint')

    @joint.setter
    def joint(self, value):
        self._add_child_element('joint', value)
