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
from .center import Center
from .name import Name


class SubMesh(XMLBase):
    _NAME = 'submesh'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        center=dict(creator=Center),
        name=dict(creator=Name)
    )

    def __init__(self):
        super(SubMesh, self).__init__()
        self.reset()

    @property
    def name(self):
        return self._get_child_element('name')

    @name.setter
    def name(self, value):
        self._add_child_element('name', value)

    @property
    def center(self):
        return self._get_child_element('center')

    @center.setter
    def center(self, value):
        self._add_child_element('center', value)

    def is_valid(self):
        if len(self.children) != 2:
            print('SubMesh must have a name and a center objects')
            return False
        if 'name' not in self.children:
            print('SubMesh has no item tagged as name')
            return False
        if 'center' not in self.children:
            print('SubMesh has no item tagged as center')
            return False
        if not isinstance(self.children['name'], Name):
            print('SubMesh element child is not of type Name')
            return False
        if not isinstance(self.children['center'], Center):
            print('SubMesh element child is not of type Center')
            return False
        is_valid = True
        for child in self.children.values():
            is_valid = is_valid and child.is_valid()
        return is_valid
