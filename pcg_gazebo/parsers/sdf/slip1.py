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

from ..types import XMLScalar


class Slip1(XMLScalar):
    """
    Force dependent slip direction 1 in collision local frame, between the
    range of [0,1].

    Args:
        default (float): Slip coefficient

    Attributes:
        value (float): Stored slip coefficient
    """
    _NAME = 'slip1'
    _TYPE = 'sdf'

    def __init__(self, default=0):
        XMLScalar.__init__(self, default)
        self._description = dict(
            ode='(float) Force dependent slip direction 1 in collision local'
                ' frame, between the range of [0,1]')

    def _set_value(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        assert value >= 0 and value <= 1
        XMLScalar._set_value(self, value)
