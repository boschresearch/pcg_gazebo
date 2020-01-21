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
from ..types import XMLVector


class FDir1(XMLVector):
    """
    3-tuple specifying direction of mu1 in the collision local reference frame

    Args:
        default (list): Direction unit vector

    Attributes:
        value (list): Stored direction unit vector
    """
    _NAME = 'fdir1'
    _TYPE = 'sdf'

    def __init__(self, default=[0, 0, 0]):
        XMLVector.__init__(self, 3)
        self.reset()

    def _set_value(self, value):
        assert isinstance(
            value, collections.Iterable), 'Input must be iterable'
        assert len(list(value)) == self._size, 'Wrong size'
        for item in value:
            assert isinstance(item, float) or isinstance(item, int), \
                'Input value must have float or integer components'
        XMLVector._set_value(self, value)
