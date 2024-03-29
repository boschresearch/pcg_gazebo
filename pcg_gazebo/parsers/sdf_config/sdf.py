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


class SDF(XMLString):
    _NAME = 'sdf'
    _TYPE = 'sdf_config'

    _ATTRIBUTES = dict(
        version='1.6'
    )

    def __init__(self, default='model.sdf'):
        super(SDF, self).__init__(default)
        self.reset()

    @property
    def version(self):
        return self.attributes['version']

    @version.setter
    def version(self, value):
        assert str(value) in ['1.4', '1.5', '1.6']
        self.attributes['version'] = str(value)
