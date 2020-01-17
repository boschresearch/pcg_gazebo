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


class Pose(XMLVector):
    _NAME = 'pose'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        frame=''
    )

    def __init__(self):
        XMLVector.__init__(self, 6)
        self.reset()

    @property
    def frame(self):
        return self.attributes['frame']

    @frame.setter
    def frame(self, value):
        assert isinstance(value, str)
        self.attributes['frame'] = value

    @property
    def pose(self):
        return self.value

    @pose.setter
    def pose(self, value):
        XMLVector._set_value(self, value)

    @property
    def x(self):
        return self.value[0]

    @x.setter
    def x(self, value):
        assert self._is_scalar(value)
        self.value[0] = float(value)

    @property
    def y(self):
        return self.value[1]

    @y.setter
    def y(self, value):
        assert self._is_scalar(value)
        self.value[1] = float(value)

    @property
    def z(self):
        return self.value[2]

    @z.setter
    def z(self, value):
        assert self._is_scalar(value)
        self.value[2] = float(value)

    @property
    def roll(self):
        return self.value[3]

    @roll.setter
    def roll(self, value):
        assert self._is_scalar(value)
        self.value[3] = float(value)

    @property
    def pitch(self):
        return self.value[4]

    @pitch.setter
    def pitch(self, value):
        assert self._is_scalar(value)
        self.value[4] = float(value)

    @property
    def yaw(self):
        return self.value[5]

    @yaw.setter
    def yaw(self, value):
        assert self._is_scalar(value)
        self.value[5] = float(value)
