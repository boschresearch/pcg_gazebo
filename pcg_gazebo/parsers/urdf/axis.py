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


class Axis(XMLBase):
    _NAME = 'axis'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        xyz='1 0 0'
    )

    def __init__(self, default=[1, 0, 0]):
        XMLBase.__init__(self)
        self.reset()
        self.xyz = default

    @property
    def xyz(self):
        value = []
        for num in self.attributes['xyz'].split():
            value.append(float(num))
        return value

    @xyz.setter
    def xyz(self, value):
        assert isinstance(value, list), 'Vector must be a list'
        assert len(value) == 3, 'Vector must have 3 elements'
        for i in range(len(value)):
            if not self._is_scalar(value[i]):
                self.log_error(
                    'Value in vector must be a number,'
                    ' type={}'.format(type(value[i])),
                    raise_exception=True,
                    exception_type=AssertionError)
            if value[i] < 0 or value[i] > 1:
                self.log_error(
                    'Value in vector should be within the'
                    ' interval [0, 1]', raise_exception=True,
                    exception_type=AssertionError)
        if sum(value) != 1:
            self.log_error('Axis should be an unit vector, vector={}'.format(
                value, sum(value)))
            return
        self.attributes['xyz'] = ' '.join(['{}'] * len(value))
        self.attributes['xyz'] = self.attributes['xyz'].format(*value)

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('xyz')
        obj.value = self.xyz
        return obj
