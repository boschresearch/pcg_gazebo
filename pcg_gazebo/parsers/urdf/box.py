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
import random
from ..types import XMLBase


class Box(XMLBase):
    _NAME = 'box'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        size='0 0 0'
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def size(self):
        value = []
        for num in self.attributes['size'].split():
            value.append(float(num))
        return value

    @size.setter
    def size(self, value):
        assert self._is_numeric_vector(value, [0, 1e16]), \
            'Invalid size vector'
        self.attributes['size'] = ' '.join(['{}'] * len(value))
        self.attributes['size'] = self.attributes['size'].format(*value)

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('box')
        obj.size = self.size
        return obj

    def random(self):
        self.size = [random.random() for _ in range(3)]
