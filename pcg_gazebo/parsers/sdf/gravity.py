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


class Gravity(XMLBase):
    _NAME = 'gravity'
    _TYPE = 'sdf'

    def __init__(self, default=True):
        super(Gravity, self).__init__()
        assert self._is_boolean(default) or self._is_numeric_vector(default)

        if isinstance(default, collections.Iterable):
            default = list(default)
            assert len(default) == 3
            assert self._is_numeric_vector(default)
        else:
            default = bool(default)
        self._default = default
        self._value = default

    def _set_value(self, value):
        if isinstance(self._default, bool):
            assert isinstance(value, bool) or value in [0, 1], \
                'Boolean value must be a boolean, 0 or 1'
            self._value = bool(value)
        else:
            if isinstance(value, collections.Iterable):
                assert len(list(value)) == 3
                for elem in value:
                    assert isinstance(elem, float) or isinstance(elem, int)
            self._value = list(value)

    def reset(self, mode=None, with_optional_elements=False):
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
