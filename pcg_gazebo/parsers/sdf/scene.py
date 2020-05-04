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
from .sky import Sky
from .ambient import Ambient
from .background import Background
from .shadows import Shadows
from .grid import Grid
from .origin_visual import OriginVisual
from .fog import Fog


class Scene(XMLBase):
    _NAME = 'scene'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        ambient=dict(creator=Ambient, default=[[0.4, 0.4, 0.4, 1]]),
        background=dict(creator=Background, default=[[0.7, 0.7, 0.7, 1]]),
        sky=dict(creator=Sky, optional=True),
        shadows=dict(creator=Shadows, default=[True], optional=True),
        grid=dict(creator=Grid, default=[True], optional=True),
        origin_visual=dict(
            creator=OriginVisual, default=[True], optional=True),
        fog=dict(creator=Fog, optional=True)
    )

    def __init__(self):
        super(Scene, self).__init__()
        self.reset()

    @property
    def sky(self):
        return self._get_child_element('sky')

    @sky.setter
    def sky(self, value):
        self._add_child_element('sky', value)

    @property
    def ambient(self):
        return self._get_child_element('ambient')

    @ambient.setter
    def ambient(self, value):
        self._add_child_element('ambient', value)

    @property
    def background(self):
        return self._get_child_element('background')

    @background.setter
    def background(self, value):
        self._add_child_element('background', value)

    @property
    def shadows(self):
        return self._get_child_element('shadows')

    @shadows.setter
    def shadows(self, value):
        self._add_child_element('shadows', value)

    @property
    def grid(self):
        return self._get_child_element('grid')

    @grid.setter
    def grid(self, value):
        self._add_child_element('grid', value)

    @property
    def origin_visual(self):
        return self._get_child_element('origin_visual')

    @origin_visual.setter
    def origin_visual(self, value):
        self._add_child_element('origin_visual', value)

    @property
    def fog(self):
        return self._get_child_element('fog')

    @fog.setter
    def fog(self, value):
        self._add_child_element('fog', value)
