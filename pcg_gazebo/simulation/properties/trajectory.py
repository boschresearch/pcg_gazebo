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
from .waypoint import Waypoint
from ...utils import is_integer, is_string, \
    is_array, is_scalar
from ...parsers.sdf import create_sdf_element, \
    is_sdf_element


class Trajectory(object):
    def __init__(self, id=0, type='__default__', tension=0, waypoints=None):
        self._id = None
        self._type = None
        self._tension = None
        self._waypoints = list()

        self.id = id
        self.type = type

        if waypoints is not None:
            assert is_array(waypoints), 'Waypoints input must be a list'
            for item in waypoints:
                print('add waypoint=', item)
                if isinstance(item, dict):
                    self.add_waypoint(**item)
                elif isinstance(item, Waypoint):
                    self._waypoints.append(item)
                elif is_sdf_element(item) and \
                        item.xml_element_name == 'waypoint':
                    self._waypoints.append(Waypoint.from_sdf(item))

    @property
    def id(self):
        return self._id

    @id.setter
    def id(self, value):
        assert is_integer(value), 'ID must be an integer'
        assert value >= 0, 'ID must be greater or equal to zero'
        self._id = value

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, value):
        assert is_string(value), 'Type must be a string'
        self._type = value

    @property
    def tension(self):
        return self._tension

    @tension.setter
    def tension(self, value):
        assert is_scalar(value), 'Tension must be a scalar'
        self._tension = value

    @property
    def num_waypoints(self):
        return len(self._waypoints)

    @property
    def waypoints(self):
        return self._waypoints

    def add_waypoint(self, **kwargs):
        if 'time' in kwargs and 'pose' in kwargs:
            self._waypoints.append(Waypoint(**kwargs))
        elif 'waypoint' in kwargs:
            assert isinstance(kwargs['waypoint'], Waypoint)
            self._waypoints.append(kwargs['waypoint'])
        else:
            raise ValueError('Invalid waypoint input')

    def get_waypoint(self, id):
        if len(self._waypoints) == 0:
            return None
        if id < 0 or id >= len(self._waypoints):
            return None
        return self._waypoints[id]

    def to_sdf(self):
        sdf = create_sdf_element('trajectory')
        sdf.id = self._id
        sdf.type = self._type
        sdf.tension = self._tension
        for wp in self._waypoints:
            sdf.add_waypoint(waypoint=wp.to_sdf())
        return sdf

    @staticmethod
    def from_sdf(sdf):
        trajectory = Trajectory(sdf.id, sdf.type, sdf.tension)
        for waypoint in sdf.waypoints:
            trajectory.add_waypoint(
                waypoint=Waypoint.from_sdf(waypoint))
        return trajectory
