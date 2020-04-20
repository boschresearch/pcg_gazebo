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
from .path import Path


class Save(XMLBase):
    _NAME = 'save'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        path=dict(creator=Path)
    )

    _ATTRIBUTES = dict(
        enabled=False
    )

    def __init__(self):
        super(Save, self).__init__()
        self.reset()

    @property
    def enabled(self):
        return self.attributes['enabled']

    @enabled.setter
    def enabled(self, value):
        assert isinstance(value, bool) or value in [0, 1], \
            'Input value must be a boolean, 0 or 1'
        self.attributes['enabled'] = bool(value)

    @property
    def path(self):
        return self._get_child_element('path')

    @path.setter
    def path(self, value):
        self._add_child_element('path', value)
