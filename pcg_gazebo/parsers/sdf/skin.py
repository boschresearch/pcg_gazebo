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
from .filename import Filename
from .scale import Scale


class Skin(XMLBase):
    _NAME = 'skin'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        filename=dict(creator=Filename, default=['__default__']),
        scale=dict(creator=Scale, default=[1], optional=True)
    )

    def __init__(self):
        super(Skin, self).__init__()
        self.reset()

    @property
    def filename(self):
        return self._get_child_element('filename')

    @filename.setter
    def filename(self, value):
        self._add_child_element('filename', value)

    @property
    def scale(self):
        return self._get_child_element('scale')

    @scale.setter
    def scale(self, value):
        self._add_child_element('scale', value)
