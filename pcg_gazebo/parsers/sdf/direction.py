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
import collections
from ..types import XMLBase


class Direction(XMLBase):
    _NAME = 'direction'
    _TYPE = 'sdf'

    def __init__(self, default=[0, 0, -1]):
        XMLBase.__init__(self)
        assert self._is_numeric_vector(default) or \
            self._is_scalar(default), \
            'Direction must be either an array or a scalar'

        if isinstance(default, collections.Iterable):
            default = list(default)
            assert len(default) == 3, \
                'Direction must have 3 components'
            assert self._is_numeric_vector(default), \
                'Direction must be a numerical vector'
        else:
            default = float(default)
        self._default = default
        self._value = default

    def _set_value(self, value):
        if isinstance(value, collections.Iterable):
            assert len(value) == 3, \
                'Direction must have 3 components'
            assert self._is_numeric_vector(value), \
                'Direction must be a numerical vector'
            self._value = list(value)
        else:
            self._value = float(value)

    def get_formatted_value_as_str(self):
        if isinstance(self._value, collections.Iterable):
            output_str = ' '.join(['{}'] * len(self._value))
            return output_str.format(*[format(x, 'n') for x in self._value])
        else:
            return '{}'.format(self._value)
