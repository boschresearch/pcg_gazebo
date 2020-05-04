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
import random


class Velocity(XMLBase):
    _NAME = 'velocity'
    _TYPE = 'sdf'

    _MODES = ['vector', 'scalar']

    def __init__(self, default=-1):
        super(Velocity, self).__init__()

        assert self._is_scalar(default) or \
            self._is_numeric_vector(default), \
            'Input default must be a scalar or a vector'

        if self._is_numeric_vector(default):
            self._mode = 'vector'
            default = list(default)
            assert len(default) == 6, 'List must have 6 elements'
            self._is_numeric_vector(default), \
                'Input default vector is not a numeric vector'
            self._default = default
            self._value = default
            self._VALUE_TYPE = 'vector'
            self._size = 6
        else:
            self._mode = 'scalar'
            self._default = default
            self._value = default
            self._VALUE_TYPE = 'scalar'

    def _set_value(self, value):
        assert self._is_scalar(value) or \
            self._is_numeric_vector(value), \
            'Input default must be a scalar or a vector'
        if self._is_numeric_vector(value):
            assert len(list(value)) == 6, 'List must have 3 components'
            self._value = list(value)
            self._mode = 'vector'
        else:
            self._value = float(value)
            self._mode = 'scalar'

    def reset(self, mode='scalar', with_optional_elements=False):
        if mode is not None:
            if mode not in self._MODES:
                self.log_error(
                    'Mode can either be scalar or vector',
                    raise_exception=True,
                    exception_type=AssertionError)
            self._mode = mode
        if self._mode == 'vector':
            self._default = [0 for _ in range(6)]
            self._VALUE_TYPE = 'vector'
        else:
            self._default = -1
            self._VALUE_TYPE = 'scalar'
        self._value = self._default

    def is_valid(self):
        if not isinstance(self._value, type(self._default)):
            self.log_error('Object must have a {}'.format(type(self._default)))
            return False
        return True

    def get_formatted_value_as_str(self):
        assert self.is_valid(), 'Invalid type'
        if self._is_numeric_vector(self._value):
            output_str = ' '.join(['{}'] * len(self._value))
            return output_str.format(*self._value)
        else:
            return '{}'.format(self._value)

    def random(self):
        if self._mode == 'scalar':
            self._set_value(random.rand())
        else:
            self._set_value([random.rand() for _ in range(self._size)])
