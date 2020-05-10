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
from .surface_model import SurfaceModel
from .world_frame_orientation import WorldFrameOrientation
from .latitude_deg import LatitudeDeg
from .longitude_deg import LongitudeDeg
from .elevation import Elevation
from .heading_deg import HeadingDeg


class SphericalCoordinates(XMLBase):
    _NAME = 'spherical_coordinates'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        surface_model=dict(creator=SurfaceModel, default=['EARTH_WGS84']),
        world_frame_orientation=dict(
            creator=WorldFrameOrientation, default=['ENU']),
        latitude_deg=dict(creator=LatitudeDeg, default=[0]),
        longitude_deg=dict(creator=LongitudeDeg, default=[0]),
        elevation=dict(creator=Elevation, default=[0]),
        heading_deg=dict(creator=HeadingDeg, default=[0])
    )

    def __init__(self):
        super(SphericalCoordinates, self).__init__()
        self.reset()

    @property
    def surface_model(self):
        return self._get_child_element('surface_model')

    @surface_model.setter
    def surface_model(self, value):
        self._add_child_element('surface_model', value)

    @property
    def world_frame_orientation(self):
        return self._get_child_element('world_frame_orientation')

    @world_frame_orientation.setter
    def world_frame_orientation(self, value):
        self._add_child_element('world_frame_orientation', value)

    @property
    def latitude_deg(self):
        return self._get_child_element('latitude_deg')

    @latitude_deg.setter
    def latitude_deg(self, value):
        self._add_child_element('latitude_deg', value)

    @property
    def longitude_deg(self):
        return self._get_child_element('longitude_deg')

    @longitude_deg.setter
    def longitude_deg(self, value):
        self._add_child_element('longitude_deg', value)

    @property
    def elevation(self):
        return self._get_child_element('elevation')

    @elevation.setter
    def elevation(self, value):
        self._add_child_element('elevation', value)

    @property
    def heading_deg(self):
        return self._get_child_element('heading_deg')

    @heading_deg.setter
    def heading_deg(self, value):
        self._add_child_element('heading_deg', value)
