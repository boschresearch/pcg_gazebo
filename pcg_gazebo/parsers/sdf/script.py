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
from .uri import URI
from .loop import Loop
from .delay_start import DelayStart
from .auto_start import AutoStart
from .trajectory import Trajectory


class Script(XMLBase):
    _NAME = 'script'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        name=dict(
            creator=Name,
            mode='material',
            default=['default']),
        uri=dict(
            creator=URI,
            mode='material',
            default=['file://media/materials/scripts/gazebo.material'],
            n_elems='+',
            optional=True),
        loop=dict(
            creator=Loop,
            mode='actor',
            default=[True],
            optional=True),
        delay_start=dict(
            creator=DelayStart,
            mode='actor',
            default=[0],
            optional=True),
        auto_start=dict(
            creator=AutoStart,
            mode='actor',
            default=[True],
            optional=True),
        trajectory=dict(
            creator=Trajectory,
            mode='actor',
            optional=True,
            n_elems='+'))

    _MODES = ['material', 'actor']

    def __init__(self, mode='material'):
        XMLBase.__init__(self)
        self.reset(mode=mode)

    @property
    def name(self):
        return self._get_child_element('name')

    @name.setter
    def name(self, value):
        self._add_child_element('name', value)

    @property
    def loop(self):
        return self._get_child_element('loop')

    @loop.setter
    def loop(self, value):
        self._add_child_element('loop', value)

    @property
    def delay_start(self):
        return self._get_child_element('delay_start')

    @delay_start.setter
    def delay_start(self, value):
        self._add_child_element('delay_start', value)

    @property
    def auto_start(self):
        return self._get_child_element('auto_start')

    @auto_start.setter
    def auto_start(self, value):
        self._add_child_element('auto_start', value)

    @property
    def trajectories(self):
        return self._get_child_element('trajectory')

    @property
    def uris(self):
        return self._get_child_element('uri')

    def add_uri(self, value):
        self._add_child_element('uri', value)

    def add_trajectory(self, value):
        self._add_child_element('trajectory', value)

    def is_valid(self):
        if self.get_mode() == 'material':
            if self._get_child_element('uri') is None:
                print('No URIs found for script')
                return False
        return XMLBase.is_valid(self)
