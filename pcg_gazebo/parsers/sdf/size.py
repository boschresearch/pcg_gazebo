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
from ..types import XMLVector


class Size(XMLVector):
    _NAME = 'size'
    _TYPE = 'sdf'

    def __init__(self, vec_length=3):
        XMLVector.__init__(self, vec_length)

    @property
    def width(self):
        return self.value[0]

    @width.setter
    def width(self, value):
        assert self._is_scalar(value)
        assert value > 0
        self.value[0] = float(value)

    @property
    def length(self):
        return self.value[1]

    @length.setter
    def length(self, value):
        assert self._is_scalar(value)
        assert value > 0
        self.value[1] = float(value)

    @property
    def height(self):
        assert self._size == 3
        return self.value[2]

    @height.setter
    def height(self, value):
        assert self._size == 3
        assert self._is_scalar(value)
        assert value > 0
        self.value[2] = float(value)

    def _set_value(self, value):
        XMLVector._set_value(self, value, min_value=0)
