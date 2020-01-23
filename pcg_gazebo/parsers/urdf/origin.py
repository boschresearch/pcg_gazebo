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


class Origin(XMLBase):
    _NAME = 'origin'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        xyz='0 0 0',
        rpy='0 0 0'
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def xyz(self):
        value = []
        for num in self.attributes['xyz'].split():
            value.append(float(num))
        return value

    @xyz.setter
    def xyz(self, value):
        assert isinstance(value, list)
        assert len(value) == 3
        for elem in value:
            assert isinstance(elem, float) or isinstance(elem, int)
        output_str = ' '.join(['{}'] * len(value))
        self.attributes['xyz'] = output_str.format(*value)

    @property
    def rpy(self):
        value = []
        for num in self.attributes['rpy'].split():
            value.append(float(num))
        return value

    @rpy.setter
    def rpy(self, value):
        assert isinstance(value, list)
        assert len(value) == 3
        for elem in value:
            assert isinstance(elem, float) or isinstance(elem, int)
        output_str = ' '.join(['{}'] * len(value))
        self.attributes['rpy'] = output_str.format(*value)

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('pose')
        obj.pose = self.xyz + self.rpy
        return obj

    def random(self):
        self.xyz = [random.random() for _ in range(3)]
        self.rpy = [random.random() for _ in range(3)]
