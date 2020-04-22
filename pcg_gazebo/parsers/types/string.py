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
from ...utils import is_string, generate_random_string


class XMLString(XMLBase):
    _NAME = ''
    _VALUE_TYPE = 'string'

    def __init__(self, default=''):
        XMLBase.__init__(self)
        assert isinstance(default, str)
        self._value = default
        self._default = default

    def _set_value(self, value):
        assert is_string(value), \
            '[{}] Input value must be string' \
            ' or unicode, received={}, type={}'.format(
                self.xml_element_name, value, type(value))
        self._value = str(value)

    def reset(self, mode=None, with_optional_elements=False):
        self._value = self._default
        XMLBase.reset(self)

    def is_valid(self):
        if not isinstance(self._value, str):
            print('Name object must have a string')
            return False
        return True

    def get_formatted_value_as_str(self):
        assert self.is_valid(), 'Invalid string'
        return '{}'.format(str(self._value))

    def random(self):
        self._set_value(generate_random_string(5))
