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


class XMLScalar(XMLBase):
    _NAME = ''

    def __init__(self, default=0):
        XMLBase.__init__(self)
        assert isinstance(default, float) or isinstance(default, int), \
            'Input default value must be either a float or an integer'
        self._default = default
        self._value = default

    def _set_value(self, value):
        assert not isinstance(value, bool), 'Input value cannot be a boolean'
        assert isinstance(value, float) or isinstance(value, int), \
            '[{}] Input value must be either a float or an integer for {},' \
            ' received={}, type={}'.format(
                self.xml_element_name, self._NAME, value, type(value))
        self._value = float(value)

    def reset(self):
        self._value = self._default
        XMLBase.reset(self)

    def is_valid(self):
        if not isinstance(self._value, float) and \
           not isinstance(self._value, int):
            print('Scalar object must have a float or integer as a value')
            return False
        return True

    def get_formatted_value_as_str(self):
        assert self.is_valid(), 'Invalid scalar value'
        return '{}'.format(self._value)
