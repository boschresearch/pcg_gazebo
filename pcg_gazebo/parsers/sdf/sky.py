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
from .time import Time
from .sunrise import Sunrise
from .sunset import Sunset
from .clouds import Clouds


class Sky(XMLBase):
    _NAME = 'sky'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        time=dict(creator=Time, default=[10], optional=True),
        sunrise=dict(creator=Sunrise, default=[6], optional=True),
        sunset=dict(creator=Sunset, default=[20], optional=True),
        clouds=dict(creator=Clouds, optional=True)
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def time(self):
        return self._get_child_element('time')

    @time.setter
    def time(self, value):
        self._add_child_element('time', value)

    @property
    def sunrise(self):
        return self._get_child_element('sunrise')

    @sunrise.setter
    def sunrise(self, value):
        self._add_child_element('sunrise', value)

    @property
    def sunset(self):
        return self._get_child_element('sunset')

    @sunset.setter
    def sunset(self, value):
        self._add_child_element('sunset', value)

    @property
    def clouds(self):
        return self._get_child_element('clouds')

    @clouds.setter
    def clouds(self, value):
        self._add_child_element('clouds', value)
