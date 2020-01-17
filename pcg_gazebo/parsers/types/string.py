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


class XMLString(XMLBase):
    _NAME = ''

    def __init__(self, default=''):
        XMLBase.__init__(self)
        assert isinstance(default, str)
        self._value = default
        self._default = default

    def _set_value(self, value):
        import sys
        if sys.version_info[0] == 2:
            assert isinstance(value, str) or isinstance(value, unicode), \
                '[{}] Input value must be string or unicode, received={}, type={}'.format(
                    self.xml_element_name, value, type(value))
        else:
            assert isinstance(value, str), \
                '[{}] Input value must be string or unicode, received={}, type={}'.format(
                    self.xml_element_name, value, type(value))
        self._value = str(value)

    def reset(self):
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
