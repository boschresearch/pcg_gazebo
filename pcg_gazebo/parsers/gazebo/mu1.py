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
from ..sdf.bullet import FrictionBullet


class Mu1(XMLScalar):
    _NAME = 'mu1'
    _TYPE = 'gazebo'

    def __init__(self, default=1):
        XMLScalar.__init__(self, default)

    def _set_value(self, value):
        assert self._is_scalar(value), \
            'Input value for {} must be a scalar'.format(self._NAME)
        assert value >= 0, \
            'Input value for {} must be equal or' \
            ' greater than zero'.format(self._NAME)
        XMLScalar._set_value(self, value)

    def to_sdf(self, engine='ode'):
        from ..sdf import create_sdf_element

        assert engine in ['ode', 'bullet'], 'Accepted engine inputs are ode' \
            ' or bullet'
        if engine == 'ode':
            obj = create_sdf_element('mu')
            obj.value = self.value
        else:
            obj = FrictionBullet(self.value)
        return obj
