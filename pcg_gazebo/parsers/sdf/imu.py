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
from .angular_velocity import AngularVelocity
from .linear_acceleration import LinearAcceleration
from .topic import Topic
from .orientation_reference_frame import OrientationReferenceFrame
from .noise import Noise


class IMU(XMLBase):
    _NAME = 'imu'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        angular_velocity=dict(
            creator=AngularVelocity,
            optional=True,
            sdf_versions=[
                '1.5',
                '1.6']),
        linear_acceleration=dict(
            creator=LinearAcceleration,
            optional=True,
            sdf_versions=[
                '1.5',
                '1.6']),
        topic=dict(
            creator=Topic,
            default=['__default_topic__'],
            optional=True),
        orientation_reference_frame=dict(
            creator=OrientationReferenceFrame,
            optional=True,
            sdf_versions=['1.6']),
        noise=dict(
            creator=Noise,
            optional=True,
            sdf_versions=[
                '1.4',
                '1.5']))

    def __init__(self):
        super(IMU, self).__init__()
        self.reset()

    @property
    def orientation_reference_frame(self):
        return self._get_child_element('orientation_reference_frame')

    @orientation_reference_frame.setter
    def orientation_reference_frame(self, value):
        self._add_child_element('orientation_reference_frame', value)

    @property
    def angular_velocity(self):
        return self._get_child_element('angular_velocity')

    @angular_velocity.setter
    def angular_velocity(self, value):
        self._add_child_element('angular_velocity', value)

    @property
    def linear_acceleration(self):
        return self._get_child_element('linear_acceleration')

    @linear_acceleration.setter
    def linear_acceleration(self, value):
        self._add_child_element('linear_acceleration', value)

    @property
    def topic(self):
        return self._get_child_element('topic')

    @topic.setter
    def topic(self, value):
        self._add_child_element('topic', value)

    @property
    def noise(self):
        return self._get_child_element('noise')

    @noise.setter
    def noise(self, value):
        self._add_child_element('noise', value)
