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
from .min_dist import MinDist
from .max_dist import MaxDist
from .static import Static
from .use_model_frame import UseModelFrame
from .xyz import XYZ
from .inherit_yaw import InheritYaw


class TrackVisual(XMLBase):
    _NAME = 'track_visual'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        name=dict(creator=Name, default=['__default__'], optional=True),
        min_dist=dict(creator=MinDist, default=[0], optional=True),
        max_dist=dict(creator=MaxDist, default=[0], optional=True),
        static=dict(creator=Static, default=[False], optional=True),
        use_model_frame=dict(
            creator=UseModelFrame, default=[True], optional=True),
        xyz=dict(creator=XYZ, default=[[-5.0, 0.0, 3.0]], optional=True),
        inherit_yaw=dict(creator=InheritYaw, default=[False], optional=True)
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def name(self):
        return self._get_child_element('name')

    @name.setter
    def name(self, value):
        self._add_child_element('name', value)

    @property
    def min_dist(self):
        return self._get_child_element('min_dist')

    @min_dist.setter
    def min_dist(self, value):
        self._add_child_element('min_dist', value)

    @property
    def max_dist(self):
        return self._get_child_element('max_dist')

    @max_dist.setter
    def max_dist(self, value):
        self._add_child_element('max_dist', value)

    @property
    def static(self):
        return self._get_child_element('static')

    @static.setter
    def static(self, value):
        self._add_child_element('static', value)

    @property
    def use_model_frame(self):
        return self._get_child_element('use_model_frame')

    @use_model_frame.setter
    def use_model_frame(self, value):
        self._add_child_element('use_model_frame', value)

    @property
    def xyz(self):
        return self._get_child_element('xyz')

    @xyz.setter
    def xyz(self, value):
        self._add_child_element('xyz', value)

    @property
    def inherit_yaw(self):
        return self._get_child_element('inherit_yaw')

    @inherit_yaw.setter
    def inherit_yaw(self, value):
        self._add_child_element('inherit_yaw', value)
