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
from .size import Size
from .diffuse import Diffuse
from .normal import Normal


class Texture(XMLBase):
    _NAME = 'texture'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        size=dict(creator=Size, default=[[10]]),
        diffuse=dict(creator=Diffuse, default=['__default__']),
        normal=dict(creator=Normal, default=['__default__'])
    )

    def __init__(self):
        super(Texture, self).__init__()
        self.reset()

    @property
    def size(self):
        return self._get_child_element('size')

    @size.setter
    def size(self, value):
        self._add_child_element('size', value)

    @property
    def diffuse(self):
        return self._get_child_element('diffuse')

    @diffuse.setter
    def diffuse(self, value):
        self._add_child_element('diffuse', value)

    @property
    def normal(self):
        return self._get_child_element('normal')

    @normal.setter
    def normal(self, value):
        self._add_child_element('normal', value)
