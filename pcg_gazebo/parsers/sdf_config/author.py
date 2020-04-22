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
from .name import Name
from .email import EMail


class Author(XMLBase):
    _NAME = 'author'
    _TYPE = 'sdf_config'

    _CHILDREN_CREATORS = dict(
        name=dict(creator=Name, default=['none']),
        email=dict(creator=EMail, default=['email@email.com'], optional=True)
    )

    def __init__(self):
        super(Author, self).__init__()
        self.reset()

    @property
    def name(self):
        return self._get_child_element('name')

    @name.setter
    def name(self, value):
        self._add_child_element('name', value)

    @property
    def email(self):
        return self._get_child_element('email')

    @email.setter
    def email(self, value):
        self._add_child_element('email', value)
