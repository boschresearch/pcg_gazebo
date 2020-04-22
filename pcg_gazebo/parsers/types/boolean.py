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

from . import XMLBase


class XMLBoolean(XMLBase):
    _NAME = ''
    _VALUE_TYPE = 'boolean'

    def __init__(self, default=False):
        XMLBase.__init__(self)
        assert isinstance(default, bool)
        self._default = default
        self._value = default

    def _set_value(self, value):
        if value in [0, 1]:
            value = bool(value)
        if value in ['true', 'false']:
            value = True if value == 'true' else False
        assert isinstance(value, bool), \
            '[{}] Input value must be a boolean, 0' \
            ' or 1, type={}, name={}, received={}'.format(
                self.xml_element_name, self._TYPE, self._NAME, type(value))
        self._value = bool(value)

    def reset(self, mode=None, with_optional_elements=False):
        self._value = self._default
        XMLBase.reset(self)

    def is_valid(self):
        if not isinstance(self._value, bool):
            print('Object must have a boolean')
            return False
        return True

    def get_formatted_value_as_str(self):
        assert self.is_valid(), 'Invalid boolean'
        return '{}'.format(int(self._value))

    def random(self):
        import random
        self._set_value(random.choice([True, False]))
