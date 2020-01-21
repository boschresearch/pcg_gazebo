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
from .far import Far
from .near import Near


class Clip(XMLBase):
    _NAME = 'clip'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        far=dict(creator=Far, default=[0.1]),
        near=dict(creator=Near, default=[100])
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def far(self):
        return self._get_child_element('far')

    @far.setter
    def far(self, value):
        self._add_child_element('far', value)

    @property
    def near(self):
        return self._get_child_element('near')

    @near.setter
    def near(self, value):
        self._add_child_element('near', value)
