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
from .uri import URI
from .pose import Pose
from .name import Name
from .static import Static


class Include(XMLBase):
    _NAME = 'include'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        uri=dict(creator=URI),
        pose=dict(creator=Pose, optional=True),
        name=dict(creator=Name, optional=True),
        static=dict(creator=Static, optional=True)
    )

    def __init__(self):
        super(Include, self).__init__()
        self.reset()

    @property
    def uri(self):
        return self._get_child_element('uri')

    @uri.setter
    def uri(self, value):
        self._add_child_element('uri', value)

    @property
    def pose(self):
        return self._get_child_element('pose')

    @pose.setter
    def pose(self, value):
        self._add_child_element('pose', value)

    @property
    def name(self):
        return self._get_child_element('name')

    @name.setter
    def name(self, value):
        self._add_child_element('name', value)

    @property
    def static(self):
        return self._get_child_element('static')

    @static.setter
    def static(self, value):
        self._add_child_element('static', value)
