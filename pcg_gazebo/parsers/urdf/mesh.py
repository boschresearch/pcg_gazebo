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
from ...utils import is_string, generate_random_string


class Mesh(XMLBase):
    _NAME = 'mesh'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        filename='',
        scale='1 1 1'
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def filename(self):
        return self.attributes['filename']

    @filename.setter
    def filename(self, value):
        assert is_string(value), \
            'Filename must be a string'
        self.attributes['filename'] = value

    @property
    def scale(self):
        value = []
        for num in self.attributes['scale'].split():
            value.append(float(num))
        return value

    @scale.setter
    def scale(self, value):
        assert isinstance(value, list), 'Input must be a list'
        assert len(value) == 3, 'Input vector must have 3 elements'
        for elem in value:
            assert isinstance(elem, float) or isinstance(elem, int)
        output_str = ' '.join(['{}'] * len(value))
        self.attributes['scale'] = output_str.format(*value)

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('mesh')
        obj.uri = self.filename
        obj.scale = self.scale
        return obj

    def random(self):
        self.scale = [random.random() for _ in range(3)]
        self.filename = generate_random_string(5)
