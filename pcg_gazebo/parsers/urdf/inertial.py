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
from .mass import Mass
from .origin import Origin
from .inertia import Inertia


class Inertial(XMLBase):
    _NAME = 'inertial'
    _TYPE = 'urdf'

    _CHILDREN_CREATORS = dict(
        mass=dict(creator=Mass),
        origin=dict(creator=Origin),
        inertia=dict(creator=Inertia)
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def mass(self):
        return self._get_child_element('mass')

    @mass.setter
    def mass(self, value):
        self._add_child_element('mass', value)

    @property
    def origin(self):
        return self._get_child_element('origin')

    @origin.setter
    def origin(self, value):
        self._add_child_element('origin', value)

    @property
    def inertia(self):
        return self._get_child_element('inertia')

    @inertia.setter
    def inertia(self, value):
        self._add_child_element('inertia', value)

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('inertial')
        obj.pose = self.origin.to_sdf()
        obj.inertia = self.inertia.to_sdf()
        obj.mass = self.mass.to_sdf()
        return obj
