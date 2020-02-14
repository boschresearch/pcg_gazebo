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


class Center(XMLBase):
    _NAME = 'center'
    _TYPE = 'sdf'

    _MODES = ['boolean', 'vector']

    def __init__(self, default=False):
        XMLBase.__init__(self)

        assert self._is_boolean(default) or self._is_array(default), \
            'Input default value must be a boolean, a list or an array'
        if self._is_array(default):
            self._mode = 'vector'
            default = list(default)
            assert len(default) == 2, 'List must have 2 elements'
            self._is_numeric_vector(default), \
                'Input default vector is not a numeric vector'
            self._default = default
            self._value = default
        else:
            self._mode = 'boolean'
            self._default = bool(default)
            self._value = bool(default)

    def _set_value(self, value):
        assert self._is_boolean(value) or self._is_array(value), \
            'Input default value must be a boolean, a list or an array'
        if self._is_numeric_vector(value):
            assert len(list(value)) == 2, 'List must have 2 elements'
            self._value = list(value)
            self._mode = 'vector'
        else:
            self._value = bool(value)
            self._mode = 'boolean'

    def reset(self, mode='boolean', with_optional_elements=False):
        if mode is not None:
            if mode not in self._MODES:
                self.log_error(
                    'Mode can either be boolean or vector',
                    raise_exception=True,
                    exception_type=AssertionError)
            self._mode = mode
        if self._mode == 'boolean':
            self._default = False
            self._VALUE_TYPE = 'boolean'
        else:
            self._default = [0, 0]
            self._VALUE_TYPE = 'vector'
        self._value = self._default

    def is_valid(self):
        if not isinstance(self._value, type(self._default)):
            print('Object must have a {}'.format(type(self._default)))
            return False
        return True

    def get_formatted_value_as_str(self):
        assert self.is_valid(), 'Invalid type'
        if isinstance(self._value, bool):
            return '{}'.format(int(self._value))
        else:
            output_str = ' '.join(['{}'] * len(self._value))
            return output_str.format(*self._value)
