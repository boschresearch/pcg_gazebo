# Copyright (c) 2020 - The Procedural Generation for Gazebo authors
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
from .rule import Rule
from ...utils import is_scalar


class FixedValue(Rule):
    _NAME = 'value'

    def __init__(self, dofs=None, value=None, **kwargs):
        assert is_scalar(value), 'Input value must be a scalar'
        self._value = value
        super(FixedValue, self).__init__(dofs=dofs)

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        assert is_scalar(value), 'Input value must be a scalar'
        self._value = value

    def _get_value(self):
        return self._value

    @staticmethod
    def example():
        sample = Rule.example()
        sample['value'] = 0
        sample['tag'] = FixedValue._NAME
        return sample
