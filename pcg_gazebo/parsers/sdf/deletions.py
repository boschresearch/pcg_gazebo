# Copyright (c) 2020 - The Procedural Generation for Gazebo authors
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
from .name import Name


class Deletions(XMLBase):
    _NAME = 'deletions'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        name=dict(creator=Name, n_elems='+', optional=True)
    )

    def __init__(self):
        super(Deletions, self).__init__()
        self.reset()

    @property
    def names(self):
        return self._get_child_element('name')

    def add_name(self, tag, name=None):
        if self.names is not None:
            for elem in self.names:
                if elem.name == name:
                    print(
                        'Name element with name {}'
                        ' already exists'.format(name))
                    return
        if name is not None:
            self._add_child_element('name', name)
        else:
            name = Name()
            self._add_child_element('name', name)
        self.children['name'][-1].name = name
