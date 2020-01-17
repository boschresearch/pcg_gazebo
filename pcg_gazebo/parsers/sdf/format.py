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

from ..types import XMLString


class Format(XMLString):
    _NAME = 'format'
    _TYPE = 'sdf'

    _VALUE_OPTIONS = ['L8', 'R8G8B8', 'B8G8R8', 'BAYER_RGGB8',
                      'BAYER_BGGR8', 'BAYER_GBRG8', 'BAYER_GRBG8']

    def __init__(self, default='R8G8B8'):
        XMLString.__init__(self, default)

    def _set_value(self, value):
        assert value in self._VALUE_OPTIONS, 'Options are {}'.format(
            self._VALUE_OPTIONS)
        XMLString._set_value(self, value)
