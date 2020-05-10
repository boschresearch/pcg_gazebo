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
from .pose import Pose


class Frame(XMLBase):
    _NAME = 'frame'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        name='__default__'
    )

    _CHILDREN_CREATORS = dict(
        pose=dict(creator=Pose, default=[[0, 0, 0, 0, 0, 0]], optional=True)
    )

    def __init__(self, default='__default__'):
        super(Frame, self).__init__()
        self.reset()

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert self._is_string(value), \
            'Input name must be a string'
        self.attributes['name'] = str(value)
