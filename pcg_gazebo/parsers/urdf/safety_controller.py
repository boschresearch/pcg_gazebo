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


class SafetyController(XMLBase):
    _NAME = 'safety_controller'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        soft_lower_limit='0',
        soft_upper_limit='0',
        k_position='0',
        k_velocity='0'
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def soft_lower_limit(self):
        return float(self.attributes['soft_lower_limit'])

    @soft_lower_limit.setter
    def soft_lower_limit(self, value):
        assert isinstance(value, float) or isinstance(value, int), \
            'Input value must be either a float or an integer'
        self.attributes['soft_lower_limit'] = float(value)

    @property
    def soft_upper_limit(self):
        return float(self.attributes['soft_upper_limit'])

    @soft_upper_limit.setter
    def soft_upper_limit(self, value):
        assert isinstance(value, float) or isinstance(value, int), \
            'Input value must be either a float or an integer'
        self.attributes['soft_upper_limit'] = float(value)

    @property
    def k_position(self):
        return float(self.attributes['k_position'])

    @k_position.setter
    def k_position(self, value):
        assert isinstance(value, float) or isinstance(value, int), \
            'Input value must be either a float or an integer'
        self.attributes['k_position'] = float(value)

    @property
    def k_velocity(self):
        return float(self.attributes['k_velocity'])

    @k_velocity.setter
    def k_velocity(self, value):
        assert isinstance(value, float) or isinstance(value, int), \
            'Input value must be either a float or an integer'
        self.attributes['k_velocity'] = float(value)
