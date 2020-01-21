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
from .ixx import IXX
from .iyy import IYY
from .izz import IZZ
from .ixy import IXY
from .ixz import IXZ
from .iyz import IYZ


class Inertia(XMLBase):
    _NAME = 'inertia'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        ixx=dict(creator=IXX),
        iyy=dict(creator=IYY),
        izz=dict(creator=IZZ),
        ixy=dict(creator=IXY),
        ixz=dict(creator=IXZ),
        iyz=dict(creator=IYZ)
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def ixx(self):
        return self._get_child_element('ixx')

    @ixx.setter
    def ixx(self, value):
        self._add_child_element('ixx', value)

    @property
    def ixy(self):
        return self._get_child_element('ixy')

    @ixy.setter
    def ixy(self, value):
        self._add_child_element('ixy', value)

    @property
    def ixz(self):
        return self._get_child_element('ixz')

    @ixz.setter
    def ixz(self, value):
        self._add_child_element('ixz', value)

    @property
    def iyy(self):
        return self._get_child_element('iyy')

    @iyy.setter
    def iyy(self, value):
        self._add_child_element('iyy', value)

    @property
    def iyz(self):
        return self._get_child_element('iyz')

    @iyz.setter
    def iyz(self, value):
        self._add_child_element('iyz', value)

    @property
    def izz(self):
        return self._get_child_element('izz')

    @izz.setter
    def izz(self, value):
        self._add_child_element('izz', value)
