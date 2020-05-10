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
from .waypoint import Waypoint


class Trajectory(XMLBase):
    _NAME = 'trajectory'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        id='0',
        type='__default__',
        tension='0'
    )

    _CHILDREN_CREATORS = dict(
        waypoint=dict(creator=Waypoint, n_elems='+', optional=True)
    )

    def __init__(self):
        super(Trajectory, self).__init__()
        self.reset()

    @property
    def id(self):
        return int(self.attributes['id'])

    @id.setter
    def id(self, value):
        assert self._is_integer(value), \
            'Input ID must be an integer, provided={}'.format(value)
        self.attributes['id'] = str(value)

    @property
    def type(self):
        return self.attributes['type']

    @type.setter
    def type(self, value):
        assert self._is_string(value), \
            'Type input must be a string or unicode,' \
            ' provided={}, type={}'.format(
                value, type(value))
        self.attributes['type'] = str(value)

    @property
    def tension(self):
        return float(self.attributes['tension'])

    @tension.setter
    def tension(self, value):
        assert self._is_scalar(value), \
            'Tension must be a scalar, provided={}'.format(
                value)
        self.attributes['tension'] = str(value)

    @property
    def waypoints(self):
        return self._get_child_element('waypoint')

    def add_waypoint(self, name=None, waypoint=None):
        if waypoint is not None:
            self._add_child_element('waypoint', waypoint)
        else:
            waypoint = Waypoint()
            self._add_child_element('waypoint', waypoint)
