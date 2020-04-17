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
from .linear import Linear
from .constant import Constant
from .quadratic import Quadratic
from .range import Range


class Attenuation(XMLBase):
    _NAME = 'attenuation'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        range=dict(creator=Range, default=['scalar', 10]),
        linear=dict(creator=Linear, default=[1, 0, 1]),
        constant=dict(creator=Constant, default=[1]),
        quadratic=dict(creator=Quadratic, default=[0])
    )

    def __init__(self):
        super(Attenuation, self).__init__()
        self.reset()

    @property
    def range(self):
        return self._get_child_element('range')

    @range.setter
    def range(self, value):
        self._add_child_element('range', value)

    @property
    def linear(self):
        return self._get_child_element('linear')

    @linear.setter
    def linear(self, value):
        self._add_child_element('linear', value)

    @property
    def constant(self):
        return self._get_child_element('constant')

    @constant.setter
    def constant(self, value):
        self._add_child_element('constant', value)

    @property
    def quadratic(self):
        return self._get_child_element('quadratic')

    @quadratic.setter
    def quadratic(self, value):
        self._add_child_element('quadratic', value)
