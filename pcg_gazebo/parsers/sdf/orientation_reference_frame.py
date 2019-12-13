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
from .localization import Localization


class OrientationReferenceFrame(XMLBase):
    _NAME = 'orientation_reference_frame'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        localization=dict(creator=Localization, default=['CUSTOM'])
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def localization(self):
        return self._get_child_element('localization')

    @localization.setter
    def localization(self, value):
        self._add_child_element('localization', value)
