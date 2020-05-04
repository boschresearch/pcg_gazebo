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
from .pose import Pose
from ...utils import is_array, is_scalar
from ...parsers.sdf import create_sdf_element


class Waypoint(object):
    def __init__(self, t=0, pose=[0, 0, 0, 0, 0, 0]):
        self._t = t
        self._pose = None
        self.pose = pose

    @property
    def t(self):
        return self._t

    @t.setter
    def t(self, value):
        assert is_scalar(value), 'Time is not a scalar'
        assert value >= 0, 'Time must be greater or equal to zero'
        self._t = value

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, vec):
        if isinstance(vec, Pose):
            self._pose = vec
        else:
            assert is_array(vec), \
                'Input pose vector must be iterable'
            assert len(vec) == 6 or len(vec) == 7, \
                'Pose must be given as position and Euler angles (x, y, z, ' \
                'roll, pitch, yaw) or position and quaternions (x, y, z, ' \
                'qx, qy, qz, qw)'
            for item in vec:
                assert isinstance(item, float) or isinstance(item, int), \
                    'All elements in pose vector must be a float or an integer'

            self._pose = Pose(pos=vec[0:3], rot=vec[3::])

    def to_sdf(self):
        sdf = create_sdf_element('waypoint')
        sdf.time = self._t
        sdf.pose = self._pose.to_sdf()
        return sdf

    @staticmethod
    def from_sdf(sdf):
        assert sdf.xml_element_name == \
            'waypoint', 'Invalid type of SDF element,' \
            ' provided={}'.format(sdf.xml_element_name)

        return Waypoint(
            t=sdf.time.value, pose=Pose.from_sdf(sdf.pose))
