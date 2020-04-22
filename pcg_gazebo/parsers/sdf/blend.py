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
from .min_height import MinHeight
from .fade_dist import FadeDist


class Blend(XMLBase):
    _NAME = 'blend'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        min_height=dict(creator=MinHeight, default=[0]),
        fade_dist=dict(creator=FadeDist, default=[0])
    )

    def __init__(self):
        super(Blend, self).__init__()
        self.reset()

    @property
    def min_height(self):
        return self._get_child_element('min_height')

    @min_height.setter
    def min_height(self, value):
        self._add_child_element('min_height', value)

    @property
    def fade_dist(self):
        return self._get_child_element('fade_dist')

    @fade_dist.setter
    def fade_dist(self, value):
        self._add_child_element('fade_dist', value)
