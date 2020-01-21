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
from lxml.etree import Element, SubElement


class XMLCustom(XMLBase):
    _NAME = ''

    def __init__(self, default=dict()):
        XMLBase.__init__(self)
        assert isinstance(default, dict), 'The default input must be a dict'
        self._default = default
        self._value = default

    def _set_value(self, value):
        assert isinstance(
            value, dict), 'Input value must be a dict,' \
            ' type={}, input={}'.format(
            self._NAME, value)
        self._value = value

    def _get_elem_as_xml(self, xml_elem, value):
        if isinstance(value, dict):
            for tag in value:
                child = Element(tag)
                self._get_elem_as_xml(child, value[tag])
                xml_elem.append(child)
        elif isinstance(value, bool) or isinstance(value, int):
            xml_elem.text = '{}'.format(int(value))
        elif isinstance(value, float) or isinstance(value, str):
            xml_elem.text = '{}'.format(value)
        elif isinstance(value, list):
            output_str = ' '.join(['{}'] * len(value))
            xml_elem.text = output_str.format(*value)
        return xml_elem

    def reset(self):
        self._value = self._default
        XMLBase.reset(self)

    def is_valid(self):
        if not isinstance(self._value, dict):
            print('Value must be a dict')
            return False
        return True

    def get_formatted_value_as_str(self):
        assert self.is_valid(), 'Invalid scalar value'
        return '{}'.format(self._value)

    def to_xml(self, root=None, version='1.6'):
        assert self.is_valid(), 'XML data is invalid'

        if root is None:
            base = Element(self._NAME, attrib=self.attributes)
        else:
            base = SubElement(root, self._NAME, attrib=self.attributes)

        self._get_elem_as_xml(base, self._value)
        return base

    def is_value(self, tag):
        return tag in self._value

    def find_values(self, pattern):
        output_tags = list()
        for tag in self._value:
            if pattern == self._value[tag]:
                output_tags.append(tag)
        return output_tags

    def replace_parameter_value(self, old_value, new_value):
        self._replace_value_in_dict(self._value, old_value, new_value)

    @staticmethod
    def _replace_value_in_dict(data, old_value, new_value):
        for tag in data:
            if isinstance(data[tag], dict):
                XMLCustom._replace_value_in_dict(
                    data[tag], old_value, new_value)
            elif data[tag] == old_value:
                data[tag] = new_value
