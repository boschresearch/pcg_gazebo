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
from ...utils import is_scalar
from . import XMLBase


class XMLScalar(XMLBase):
    _NAME = ''
    _VALUE_TYPE = 'scalar'

    def __init__(self, default=0, min_value=None, max_value=None):
        XMLBase.__init__(self, min_value=min_value, max_value=max_value)
        assert isinstance(default, float) or isinstance(default, int), \
            'Input default value must be either a float or an integer'
        self._default = default
        self._value = default

    def _set_value(self, value):
        assert not isinstance(value, bool), 'Input value cannot be a boolean'
        assert is_scalar(value), \
            '[{}] Input value must be either a float or an integer for {},' \
            ' received={}, type={}'.format(
                self.xml_element_name, self._NAME, value, type(value))

        if self._min_value is not None:
            assert value >= self._min_value, \
                '[{}] Value must be greater or equal to {}'.format(
                    self._NAME, self._min_value)

        if self._max_value is not None:
            if self._min_value is not None:
                assert self._max_value > self._min_value, \
                    '[{}] Max. value {} is not greater than' \
                    ' provided min. value {}'.format(
                        self._NAME, self._max_value, self._min_value)
            assert value <= self._max_value, \
                '[{}] Value must be less or equal to {}'.format(
                    self._NAME, self._max_value)

        self._value = float(value)

    def reset(self):
        self._value = self._default
        XMLBase.reset(self)

    def is_valid(self):
        if not self._is_scalar(self._value):
            self.log_error(
                'Scalar object must have a float or integer as a value')
            return False
        return True

    def get_formatted_value_as_str(self):
        assert self.is_valid(), 'Invalid scalar value'
        return '{}'.format(self._value)

    def random(self):
        import random
        self._set_value(random.random())
