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
import collections
from shapely import affinity, ops
from shapely.geometry import Point, MultiPoint
from copy import deepcopy


class Footprint(object):
    def __init__(self):
        self._polygons = list()
        self._points = None
        self._offset = [0, 0]
        self._heading = 0.0
        self._original_footprint = None

    def add_polygon(self, polygon):
        self._polygons.append(polygon)

    def add_circle(self, center, radius):
        assert isinstance(center, collections.Iterable), \
            'Center must be a list or an array'
        center = list(center)
        assert len(center) == 2, \
            'Center of the circle must have 2 elements'
        assert radius > 0, 'Radius must be greater than zero'
        self._polygons.append(Point(*center).buffer(radius))

    def add_points(self, pnts):
        self._points = pnts

    def get_footprint_polygon(self, offset=[0, 0], angle=0):
        if len(self._polygons) > 1:
            footprint = ops.cascaded_union(self._polygons)
        elif len(self._polygons) == 1:
            footprint = self._polygons[0]
        elif self._points is not None:
            mp = MultiPoint([(self._points[i, 0], self._points[i, 1])
                             for i in range(self._points.shape[0])])
            footprint = mp.convex_hull
        else:
            return None

        self._offset = offset
        self._heading = angle

        self._original_footprint = deepcopy(footprint)

        if not footprint.is_empty:
            footprint = affinity.rotate(
                footprint, self._heading, use_radians=True)
            footprint = affinity.translate(
                footprint, self._offset[0], self._offset[1], 0)

        return footprint

    def set_offset(self, offset):
        assert isinstance(offset, collections.Iterable), \
            'Offset must be a list or an array'
        offset = list(offset)
        assert len(offset) == 2, \
            'Offset must have 2 elements'
        self._offset = offset
