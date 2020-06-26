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
from .uri import URI
from .size import Size
from .pos import Pos
from .texture import Texture
from .use_terrain_paging import UseTerrainPaging
from .sampling import Sampling
from .blend import Blend


class Heightmap(XMLBase):
    _NAME = 'heightmap'
    _TYPE = 'sdf'

    _MODES = ['visual', 'collision']

    _CHILDREN_CREATORS = dict(
        uri=dict(
            creator=URI,
            default=['__default__'],
            mode=['collision', 'visual']),
        size=dict(
            creator=Size,
            default=[[1, 1, 1]],
            mode=['collision', 'visual'],
            optional=True),
        pos=dict(
            creator=Pos,
            default=[[0, 0, 0]],
            optional=True,
            mode=['collision', 'visual']),
        texture=dict(
            creator=Texture,
            n_elems='+',
            mode='visual',
            optional=True),
        blend=dict(creator=Blend, n_elems='+', mode='visual', optional=True),
        use_terrain_paging=dict(
            creator=UseTerrainPaging, default=[False],
            optional=True, mode='visual'),
        sampling=dict(
            creator=Sampling, default=[2],
            optional=True, mode='visual')
    )

    def __init__(self, mode='collision'):
        super(Heightmap, self).__init__()
        self.reset(mode=mode)

    @property
    def uri(self):
        return self._get_child_element('uri')

    @uri.setter
    def uri(self, value):
        self._add_child_element('uri', value)

    @property
    def size(self):
        return self._get_child_element('size')

    @size.setter
    def size(self, value):
        self._add_child_element('size', value)

    @property
    def pos(self):
        return self._get_child_element('pos')

    @pos.setter
    def pos(self, value):
        self._add_child_element('pos', value)

    @property
    def use_terrain_paging(self):
        return self._get_child_element('use_terrain_paging')

    @use_terrain_paging.setter
    def use_terrain_paging(self, value):
        if self._mode != 'visual':
            self._mode = 'visual'
        self._add_child_element('use_terrain_paging', value)

    @property
    def sampling(self):
        return self._get_child_element('sampling')

    @sampling.setter
    def sampling(self, value):
        if self._mode != 'visual':
            self._mode = 'visual'
        self._add_child_element('sampling', value)

    @property
    def textures(self):
        return self._get_child_element('texture')

    def add_texture(self, name=None, texture=None):
        if self._mode != 'visual':
            self._mode = 'visual'
        self._add_child_element('texture', texture)

    @property
    def blends(self):
        return self._get_child_element('blend')

    def add_blend(self, name=None, blend=None):
        if self._mode != 'visual':
            self._mode = 'visual'
        self._add_child_element('blend', blend)
