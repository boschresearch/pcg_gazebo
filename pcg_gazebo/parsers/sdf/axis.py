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
from .xyz import XYZ
from .limit import Limit
from .initial_position import InitialPosition
from .use_parent_model_frame import UseParentModelFrame
from .dynamics import Dynamics


class Axis(XMLBase):
    _NAME = 'axis'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        xyz=dict(creator=XYZ, default=[[0, 0, 1]]),
        limit=dict(creator=Limit),
        initial_position=dict(
            creator=InitialPosition, default=[0], optional=True),
        use_parent_model_frame=dict(
            creator=UseParentModelFrame, default=[False], optional=True),
        dynamics=dict(creator=Dynamics, optional=True)
    )

    def __init__(self):
        super(Axis, self).__init__()
        self.reset()

    @property
    def xyz(self):
        return self._get_child_element('xyz')

    @xyz.setter
    def xyz(self, value):
        self._add_child_element('xyz', value)

    @property
    def limit(self):
        return self._get_child_element('limit')

    @limit.setter
    def limit(self, value):
        self._add_child_element('limit', value)

    @property
    def initial_position(self):
        return self._get_child_element('initial_position')

    @initial_position.setter
    def initial_position(self, value):
        self._add_child_element('initial_position', value)

    @property
    def use_parent_model_frame(self):
        return self._get_child_element('use_parent_model_frame')

    @use_parent_model_frame.setter
    def use_parent_model_frame(self, value):
        self._add_child_element('use_parent_model_frame', value)

    @property
    def dynamics(self):
        return self._get_child_element('dynamics')

    @dynamics.setter
    def dynamics(self, value):
        self._add_child_element('dynamics', value)
