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
from .x import X
from .y import Y
from .z import Z


class LinearAcceleration(XMLBase):
    _NAME = 'linear_acceleration'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        x=dict(creator=X, optional=True),
        y=dict(creator=Y, optional=True),
        z=dict(creator=Z, optional=True)
    )

    def __init__(self):
        super(LinearAcceleration, self).__init__()
        self.reset()

    @property
    def x(self):
        return self._get_child_element('x')

    @x.setter
    def x(self, value):
        self._add_child_element('x', value)

    @property
    def y(self):
        return self._get_child_element('y')

    @y.setter
    def y(self, value):
        self._add_child_element('y', value)

    @property
    def z(self):
        return self._get_child_element('z')

    @z.setter
    def z(self, value):
        self._add_child_element('z', value)
