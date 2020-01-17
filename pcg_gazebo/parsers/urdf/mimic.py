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


class Mimic(XMLBase):
    _NAME = 'mimic'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        multiplier='1',
        offset='0'
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def multiplier(self):
        return float(self.attributes['multiplier'])

    @multiplier.setter
    def multiplier(self, value):
        assert isinstance(value, float) or isinstance(value, int), \
            'Input value must be either a float or an integer'
        self.attributes['multiplier'] = float(value)

    @property
    def offset(self):
        return float(self.attributes['offset'])

    @offset.setter
    def offset(self, value):
        assert isinstance(value, float) or isinstance(value, int), \
            'Input value must be either a float or an integer'
        self.attributes['offset'] = float(value)
