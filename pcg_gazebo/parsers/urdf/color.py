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

import collections
from ..types import XMLBase


class Color(XMLBase):
    _NAME = 'color'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        rgba='0 0 0 1'
    )

    def __init__(self, default=[0, 0, 0, 1]):
        XMLBase.__init__(self)
        self.reset()
        self.rgba = default

    @property
    def rgba(self):
        value = []
        for num in self.attributes['rgba'].split():
            value.append(float(num))
        return value

    @rgba.setter
    def rgba(self, value):
        assert isinstance(value, collections.Iterable), \
            'Input must be iterable'
        assert len(value) == 4, 'Color vector must have 4 components'
        for elem in value:
            assert isinstance(elem, float) or isinstance(elem, int), \
                'Each vector element must be either a float or an integer'
        output_str = ' '.join(['{}'] * len(value))
        self.attributes['rgba'] = output_str.format(*value)

    def to_sdf(self):
        raise NotImplementedError('Color must be converted by material')
