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
import numpy as np
from .rule import Rule
from ...utils import is_array, is_scalar


class FromSet(Rule):
    _NAME = 'from_set'

    def __init__(self, dofs=None, values=None):
        assert is_array(values), \
            'List of values must be a vector, received={}'.format(
                values)

        self._values = values
        super(FromSet, self).__init__(dofs=dofs)

    @property
    def values(self):
        return self._values

    @values.setter
    def values(self, values):
        assert is_array(values), \
            'List of values must be a vector, received={}'.format(
                values)
        for elem in values:
            assert is_scalar(elem), \
                'Vector element is not a scalar, ' \
                'received={}'.format(elem)
        self._values = values

    def _get_value(self):
        return self._values[np.random.choice(
            [i for i in range(len(self._values))])]

    @staticmethod
    def example():
        sample = Rule.example()
        sample['values'] = list()
        sample['tag'] = FromSet._NAME
        return sample
