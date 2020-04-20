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
from .scale import Scale
from .threshold import Threshold
from .granularity import Granularity
from .height import Height
from .width import Width
from .format import Format


class Image(XMLBase):
    _NAME = 'image'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        uri=dict(creator=URI, mode='geometry'),
        scale=dict(creator=Scale, default=[1], mode='geometry'),
        threshold=dict(creator=Threshold, mode='geometry'),
        granularity=dict(creator=Granularity, mode='geometry'),
        height=dict(creator=Height),
        width=dict(creator=Width, mode='camera'),
        format=dict(creator=Format, mode='camera', optional=True)
    )

    _MODES = ['geometry', 'camera']

    def __init__(self, mode='geometry'):
        super(Image, self).__init__()
        self.reset(mode=mode)

    @property
    def uri(self):
        return self._get_child_element('uri')

    @uri.setter
    def uri(self, value):
        self._add_child_element('uri', value)

    @property
    def scale(self):
        return self._get_child_element('scale')

    @scale.setter
    def scale(self, value):
        self._add_child_element('scale', value)

    @property
    def threshold(self):
        return self._get_child_element('threshold')

    @threshold.setter
    def threshold(self, value):
        self._add_child_element('threshold', value)

    @property
    def granularity(self):
        return self._get_child_element('granularity')

    @granularity.setter
    def granularity(self, value):
        self._add_child_element('granularity', value)

    @property
    def height(self):
        return self._get_child_element('height')

    @height.setter
    def height(self, value):
        self._add_child_element('height', value)

    @property
    def width(self):
        return self._get_child_element('width')

    @width.setter
    def width(self, value):
        self._add_child_element('width', value)

    @property
    def format(self):
        return self._get_child_element('format')

    @format.setter
    def format(self, value):
        self._add_child_element('format', value)
