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
from ...utils import generate_random_string
from ... import random


class Diffuse(XMLBase):
    _NAME = 'diffuse'
    _TYPE = 'sdf'

    _MODES = ['vector', 'string']

    def __init__(self, default=[0, 0, 0, 1]):
        super(Diffuse, self).__init__()

        assert self._is_numeric_vector(default) or \
            self._is_string(default), \
            'Input default value must be a string or an array'
        if self._is_numeric_vector(default):
            self._default = default
            self._value = default
            self._size = 4
        else:
            self._default = default
            self._value = default

    def _set_value(self, value):
        assert self._is_numeric_vector(value) or self._is_string(value), \
            'Input default value must be a string or an array'
        if self._is_numeric_vector(value):
            assert self._is_numeric_vector(value, [0, 1]), \
                'List must contain elements in the range [0, 1]'
            assert len(value) == 4, 'List must have 4 elements'
            self._value = list(value)
            self._mode = 'vector'
        else:
            self._value = value
            self._mode = 'string'

    def reset(self, mode='vector', with_optional_elements=False):
        if mode is not None:
            if mode not in self._MODES:
                self.log_error(
                    'Mode can either be boolean or vector',
                    raise_exception=True,
                    exception_type=AssertionError)
            self._mode = mode
        if self._mode == 'vector':
            self._default = [0, 0, 0, 1]
            self._VALUE_TYPE = 'vector'
        else:
            self._default = '__default__'
            self._VALUE_TYPE = 'string'
        self._value = self._default

    def is_valid(self):
        if not isinstance(self._value, type(self._default)):
            self.log_error('Object must have a {}'.format(type(self._default)))
            return False
        return True

    def get_formatted_value_as_str(self):
        assert self.is_valid(), 'Invalid type'
        if self._is_string(self._value):
            return '{}'.format(str(self._value))
        else:
            output_str = ' '.join(['{}'] * len(self._value))
            return output_str.format(*self._value)

    def random(self):
        if self._is_string(self._value):
            self._set_value(generate_random_string(5))
        else:
            self._set_value([random.rand() for _ in range(self._size)])
