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
from .color import Color
from .type import Type
from .start import Start
from .end import End
from .density import Density


class Fog(XMLBase):
    _NAME = 'fog'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        color=dict(creator=Color, default=[[1, 1, 1, 1]], optional=True),
        type=dict(creator=Type, default=['none'], optional=True),
        start=dict(creator=Start, default=[1.0], optional=True),
        end=dict(creator=End, default=[100.0], optional=True),
        density=dict(creator=Density, default=[1.0], optional=True)
    )

    def __init__(self, mode=''):
        super(Fog, self).__init__()
        self.reset(mode=mode)

    @property
    def color(self):
        return self._get_child_element('color')

    @color.setter
    def color(self, value):
        self._add_child_element('color', value)

    @property
    def type(self):
        return self._get_child_element('type')

    @type.setter
    def type(self, value):
        fog_types = ['constant', 'linear', 'quadratic']
        assert value in fog_types, \
            'Fog type must be one of the options={}'.format(fog_types)
        self._add_child_element('type', value)

    @property
    def start(self):
        return self._get_child_element('start')

    @start.setter
    def start(self, value):
        self._add_child_element('start', value)

    @property
    def end(self):
        return self._get_child_element('end')

    @end.setter
    def end(self, value):
        self._add_child_element('end', value)

    @property
    def density(self):
        return self._get_child_element('density')

    @density.setter
    def density(self, value):
        self._add_child_element('density', value)
