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
from .normal_map import NormalMap


class Shader(XMLBase):
    _NAME = 'shader'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        normal_map=dict(creator=NormalMap, default=['default'])
    )

    _ATTRIBUTES = dict(
        type='pixel'
    )

    def __init__(self):
        super(Shader, self).__init__()
        self.reset()

    @property
    def type(self):
        return self.attributes['type']

    @type.setter
    def type(self, value):
        assert isinstance(value, str)
        assert len(value) > 0
        assert value in [
            'vertex',
            'pixel',
            'normal_map_objectspace',
            'normal_map_tangentspace']
        self.attributes['type'] = value

    @property
    def normal_map(self):
        return self._get_child_element('normal_map')

    @normal_map.setter
    def normal_map(self, value):
        assert isinstance(value, str)
        assert len(value) > 0
        self._add_child_element('normal_map', value)
