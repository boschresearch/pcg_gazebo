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
from ...utils import generate_random_string


class Child(XMLBase):
    _NAME = 'child'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        link='link'
    )

    def __init__(self, default='link'):
        XMLBase.__init__(self)
        self.reset()

    @property
    def link(self):
        return self.attributes['link']

    @link.setter
    def link(self, value):
        assert isinstance(value, str), 'Link name must be a string'
        assert len(value) > 0, 'Link name string cannot be empty'
        self.attributes['link'] = value

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('child')
        obj.value = self.link
        return obj

    def random(self):
        self.link = generate_random_string(5)
