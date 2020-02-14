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
from .damping import Damping
from .friction import Friction
from .spring_reference import SpringReference
from .spring_stiffness import SpringStiffness


class Dynamics(XMLBase):
    _NAME = 'dynamics'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        damping=dict(creator=Damping, default=[0], optional=True),
        friction=dict(
            creator=Friction, default=['scalar', 0, 0],
            optional=True),
        spring_reference=dict(creator=SpringReference, default=[0]),
        spring_stiffness=dict(creator=SpringStiffness, default=[0])
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def damping(self):
        return self._get_child_element('damping')

    @damping.setter
    def damping(self, value):
        self._add_child_element('damping', value)

    @property
    def friction(self):
        return self._get_child_element('friction')

    @friction.setter
    def friction(self, value):
        self._add_child_element('friction', value)

    @property
    def spring_reference(self):
        return self._get_child_element('spring_reference')

    @spring_reference.setter
    def spring_reference(self, value):
        self._add_child_element('spring_reference', value)

    @property
    def spring_stiffness(self):
        return self._get_child_element('spring_stiffness')

    @spring_stiffness.setter
    def spring_stiffness(self, value):
        self._add_child_element('spring_stiffness', value)
