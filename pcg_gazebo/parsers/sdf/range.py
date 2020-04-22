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
from ... import random
from ..types import XMLBase
from .min import Min
from .max import Max
from .resolution import Resolution


class Range(XMLBase):
    _NAME = 'range'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        min=dict(creator=Min, default=[0], mode='ray'),
        max=dict(creator=Max, default=[0], mode='ray'),
        resolution=dict(creator=Resolution, default=[0], mode='ray')
    )

    _MODES = ['scalar', 'ray']

    def __init__(self, mode='ray', default=10, min_value=0):
        super(Range, self).__init__(min_value=min_value)
        if mode == 'scalar':
            self._default = default
            self._value = default
            self._VALUE_TYPE = 'scalar'
        else:
            self._default = 1.0

        self.reset(mode=mode)

    @property
    def min(self):
        return self._get_child_element('min')

    @min.setter
    def min(self, value):
        self._add_child_element('min', value)

    @property
    def max(self):
        return self._get_child_element('max')

    @max.setter
    def max(self, value):
        self._add_child_element('max', value)

    @property
    def resolution(self):
        return self._get_child_element('resolution')

    @resolution.setter
    def resolution(self, value):
        self._add_child_element('resolution', value)

    def reset(self, mode=None, with_optional_elements=False):
        if mode is not None:
            if mode not in self._MODES:
                self.log_error(
                    'Mode can either be boolean or vector',
                    raise_exception=True,
                    exception_type=AssertionError)
            self._mode = mode
        if self._mode == 'scalar':
            self.children = dict()
            self._value = self._default
            self._VALUE_TYPE = 'scalar'
        else:
            self._VALUE_TYPE = ''
            self._value = None
            XMLBase.reset(
                self, mode=mode,
                with_optional_elements=with_optional_elements)

    def _set_value(self, value):
        if self._mode != 'scalar':
            self.reset(mode='scalar')
        assert not isinstance(value, bool), 'Input value cannot be a boolean'
        assert self._is_scalar(value), \
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

    def is_valid(self):
        if self._mode == 'scalar':
            if not self._is_scalar(self._value):
                self.log_error(
                    'Scalar object must have a float or integer as a value')
                return False
            else:
                return True
        else:
            return XMLBase.is_valid(self)

    def get_formatted_value_as_str(self):
        if self._mode == 'scalar':
            assert self.is_valid(), 'Invalid scalar value'
            return '{}'.format(self._value)
        return None

    def random(self):
        if self._mode == 'scalar':
            self._set_value(random.rand())
        else:
            XMLBase.random(self)
