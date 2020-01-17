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
from .frame import Frame
from .measure_direction import MeasureDirection


class ForceTorque(XMLBase):
    _NAME = 'force_torque'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        frame=dict(
            creator=Frame,
            default=['child']),
        measure_direction=dict(
            creator=MeasureDirection,
            default=['child_to_parent']))

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def frame(self):
        return self._get_child_element('frame')

    @frame.setter
    def frame(self, value):
        self._add_child_element('frame', value)

    @property
    def measure_direction(self):
        return self._get_child_element('measure_direction')

    @measure_direction.setter
    def measure_direction(self, value):
        self._add_child_element('measure_direction', value)
