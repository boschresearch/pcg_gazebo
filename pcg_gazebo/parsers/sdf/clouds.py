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
from .ambient import Ambient
from .speed import Speed
from .direction import Direction
from .humidity import Humidity
from .mean_size import MeanSize


class Clouds(XMLBase):
    _NAME = 'clouds'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        ambient=dict(
            creator=Ambient, default=[[0.8, 0.8, 0.8, 1.0]], optional=True),
        speed=dict(creator=Speed, default=[0.6], optional=True),
        direction=dict(creator=Direction, default=[0.0], optional=True),
        humidity=dict(creator=Humidity, default=[0.5], optional=True),
        mean_size=dict(creator=MeanSize, default=[0.5], optional=True)
    )

    def __init__(self):
        super(Clouds, self).__init__()
        self.reset()

    @property
    def ambient(self):
        return self._get_child_element('ambient')

    @ambient.setter
    def ambient(self, value):
        self._add_child_element('ambient', value)

    @property
    def speed(self):
        return self._get_child_element('speed')

    @speed.setter
    def speed(self, value):
        self._add_child_element('speed', value)

    @property
    def direction(self):
        return self._get_child_element('direction')

    @direction.setter
    def direction(self, value):
        self._add_child_element('direction', value)

    @property
    def humidity(self):
        return self._get_child_element('humidity')

    @humidity.setter
    def humidity(self, value):
        self._add_child_element('humidity', value)

    @property
    def mean_size(self):
        return self._get_child_element('mean_size')

    @mean_size.setter
    def mean_size(self, value):
        self._add_child_element('mean_size', value)
