# Copyright (c) 2020 - The Procedural Generation for Gazebo authors
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
from ... import random
from .rule import Rule
from ...utils import is_scalar


class Random(Rule):
    _NAME = 'random'

    def __init__(self, dofs=None, scaling_factor=1, offset=0):
        assert is_scalar(scaling_factor), \
            'Scaling factor must be a scalar,' \
            ' received={}'.format(scaling_factor)
        assert is_scalar(offset), \
            'Offset must be a scalar, received={}'.format(
                offset)
        self._scaling_factor = scaling_factor
        self._offset = offset
        super(Random, self).__init__(dofs)

    @property
    def scaling_factor(self):
        return self._scaling_factor

    @scaling_factor.setter
    def scaling_factor(self, value):
        assert is_scalar(value), \
            'Scaling factor must be a scalar,' \
            ' received={}'.format(value)
        self._scaling_factor = value

    @property
    def offset(self):
        return self._offset

    @offset.setter
    def offset(self, value):
        assert is_scalar(value), \
            'Offset must be a scalar, received={}'.format(
                value)
        self._offset = value

    def _get_value(self):
        return self._scaling_factor * random.rand() + self._offset

    @staticmethod
    def example():
        sample = Rule.example()
        sample['scaling_factor'] = 1
        sample['offset'] = 0
        sample['tag'] = Random._NAME
        return sample
