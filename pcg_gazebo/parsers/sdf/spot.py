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
from .inner_angle import InnerAngle
from .outer_angle import OuterAngle
from .falloff import FallOff


class Spot(XMLBase):
    _NAME = 'spot'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        inner_angle=dict(creator=InnerAngle, default=[0]),
        outer_angle=dict(creator=OuterAngle, default=[0]),
        falloff=dict(creator=FallOff, default=[0])
    )

    def __init__(self):
        super(Spot, self).__init__()
        self.reset()

    @property
    def inner_angle(self):
        return self._get_child_element('inner_angle')

    @inner_angle.setter
    def inner_angle(self, value):
        self._add_child_element('inner_angle', value)

    @property
    def outer_angle(self):
        return self._get_child_element('outer_angle')

    @outer_angle.setter
    def outer_angle(self, value):
        self._add_child_element('outer_angle', value)

    @property
    def falloff(self):
        return self._get_child_element('falloff')

    @falloff.setter
    def falloff(self, value):
        self._add_child_element('falloff', value)
