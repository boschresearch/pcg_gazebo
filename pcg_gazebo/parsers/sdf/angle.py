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


class Angle(XMLBase):
    _NAME = 'angle'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        axis='0'
    )

    def __init__(self, default=0):
        super(Angle, self).__init__()
        self.reset()

    @property
    def axis(self):
        return self.attributes['axis']

    @axis.setter
    def axis(self, value):
        assert self._is_scalar(value) and \
            value >= 0, \
            'Invalid axis input, provided={}'.format(
                value)
        if isinstance(value, float):
            assert value.is_integer(), \
                'Input must be an integer, provided={}'.format(
                    value)
        self.attributes['axis'] = int(value)
