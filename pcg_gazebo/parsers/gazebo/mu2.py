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


class Mu2(XMLScalar):
    _NAME = 'mu2'
    _TYPE = 'gazebo'

    def __init__(self, default=1):
        XMLScalar.__init__(self, default, min_value=0)

    def to_sdf(self, engine='ode'):
        from ..sdf import create_sdf_element

        assert engine in ['ode', 'bullet'], 'Accepted engine inputs are ode' \
            ' or bullet'
        if engine == 'ode':
            obj = create_sdf_element('mu2')
        else:
            obj = create_sdf_element('friction2')
        obj.value = self.value
        return obj
