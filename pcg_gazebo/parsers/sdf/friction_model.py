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


class FrictionModel(XMLString):
    _NAME = 'friction_model'
    _TYPE = 'sdf'

    _VALUE_OPTIONS = ['pyramid_model', 'box_model', 'cone_model']

    def __init__(self):
        XMLString.__init__(self, default='pyramid_model')

    def _set_value(self, value):
        assert isinstance(value, str)
        assert value in self._VALUE_OPTIONS, 'Options' \
            ' are {}, received={}'.format(
                self._VALUE_OPTIONS, value)
        XMLString._set_value(self, value)
