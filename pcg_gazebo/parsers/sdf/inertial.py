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
from .inertia import Inertia
from .pose import Pose
from .mass import Mass


class Inertial(XMLBase):
    _NAME = 'inertial'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        inertia=dict(creator=Inertia),
        pose=dict(creator=Pose),
        mass=dict(creator=Mass)
    )

    def __init__(self):
        super(Inertial, self).__init__()
        self.reset()

    @property
    def mass(self):
        return self._get_child_element('mass')

    @mass.setter
    def mass(self, value):
        self._add_child_element('mass', value)

    @property
    def inertia(self):
        return self._get_child_element('inertia')

    @inertia.setter
    def inertia(self, value):
        self._add_child_element('inertia', value)

    @property
    def pose(self):
        return self._get_child_element('pose')

    @pose.setter
    def pose(self, vec):
        self._add_child_element('pose', vec)
