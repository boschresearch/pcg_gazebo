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
from .k1 import K1
from .k2 import K2
from .k3 import K3
from .p1 import P1
from .p2 import P2
from .center import Center


class Distortion(XMLBase):
    _NAME = 'distortion'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        k1=dict(creator=K1, optional=True),
        k2=dict(creator=K2, optional=True),
        k3=dict(creator=K3, optional=True),
        p1=dict(creator=P1, optional=True),
        p2=dict(creator=P2, optional=True),
        center=dict(creator=Center, default=[[0.5, 0.5]], optional=True)
    )

    def __init__(self):
        super(Distortion, self).__init__()
        self.reset()

    @property
    def k1(self):
        return self._get_child_element('k1')

    @k1.setter
    def k1(self, value):
        self._add_child_element('k1', value)

    @property
    def k2(self):
        return self._get_child_element('k2')

    @k2.setter
    def k2(self, value):
        self._add_child_element('k2', value)

    @property
    def k3(self):
        return self._get_child_element('k3')

    @k3.setter
    def k3(self, value):
        self._add_child_element('k3', value)

    @property
    def p1(self):
        return self._get_child_element('p1')

    @p1.setter
    def p1(self, value):
        self._add_child_element('p1', value)

    @property
    def p2(self):
        return self._get_child_element('p2')

    @p2.setter
    def p2(self, value):
        self._add_child_element('p2', value)

    @property
    def center(self):
        return self._get_child_element('center')

    @center.setter
    def center(self, value):
        self._add_child_element('center', value)
