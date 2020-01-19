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
from ...utils import is_string
from .waypoint import Waypoint


class Trajectory(XMLBase):
    _NAME = 'trajectory'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        id=0,
        type='__default__'
    )

    _CHILDREN_CREATORS = dict(
        waypoint=dict(creator=Waypoint, n_elems='+')
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def id(self):
        return self.attributes['id']

    @id.setter
    def id(self, value):
        assert isinstance(value, int), \
            'Input ID must be an integer, provided={}'.format(value)
        self.attributes['id'] = value

    @property
    def type(self):
        return self.attributes['type']

    @type.setter
    def type(self, value):
        assert is_string(value), \
            'Type input must be a string or unicode,' \
            ' provided={}, type={}'.format(
                value, type(value))
        self.attributes['type'] = value

    @property
    def waypoints(self):
        return self._get_child_element('waypoints')
