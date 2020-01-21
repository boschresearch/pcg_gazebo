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
import itertools
from .pose import Pose
import numpy as np


class BoundingBox(object):
    def __init__(self, min_corner=[0, 0, 0], max_corner=[0, 0, 0]):
        self._min = None
        self._max = None
        self.set_bbox(min_corner, max_corner)

    @property
    def min(self):
        return self._min

    @property
    def max(self):
        return self._max

    def set_bbox(self, min_corner=[0, 0, 0], max_corner=[0, 0, 0]):
        assert isinstance(min_corner, collections.Iterable), \
            'Min. corner point must be iterable'
        assert isinstance(max_corner, collections.Iterable), \
            'Max. corner point must be iterable'
        min_corner = list(min_corner)
        max_corner = list(max_corner)
        assert len(min_corner) == 3, 'Min. corner point must be iterable'
        assert len(max_corner) == 3, 'Max. corner point must be iterable'

        for i in range(3):
            assert min_corner[i] <= max_corner[i], \
                'Max. corner element must be greater ' \
                'than or equal to min. corner element, i={}'.format(i)
        self._min = min_corner
        self._max = max_corner

    def get_center(self, position_offset=[0, 0, 0]):
        return [self._min[i] + (self._max[i] - self._min[i]) /
                2 + position_offset[i] for i in range(3)]

    def get_corners(self, position_offset=[0, 0, 0], rotation=[1, 0, 0, 0]):
        coord = list()
        for i in range(3):
            coord.append([self._min[i], self._max[i]])

        pose = Pose(pos=position_offset, rot=rotation)
        center = self.get_center()
        corners = list()
        for item in itertools.product(*coord):
            offset_corner = list(item)
            # Remove center offset
            corner = [offset_corner[i] - center[i] for i in range(3)]
            # Apply rotation
            corner = pose.quat.rotate(corner)
            # Translate back to center and add position offset
            corner = [corner[i] + center[i] + pose.position[i]
                      for i in range(3)]

            corners.append(list(corner))

        return corners

    def get_bounds(self, position_offset=[0, 0, 0], rotation=[1, 0, 0, 0]):
        corners = self.get_corners(position_offset, rotation)

        bounds = dict(
            lower_x=np.min([c[0] for c in corners]),
            upper_x=np.max([c[0] for c in corners]),
            lower_y=np.min([c[1] for c in corners]),
            upper_y=np.max([c[1] for c in corners]),
            lower_z=np.min([c[2] for c in corners]),
            upper_z=np.max([c[2] for c in corners])
        )

        return bounds
