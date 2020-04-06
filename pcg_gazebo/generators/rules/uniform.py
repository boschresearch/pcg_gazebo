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


class Uniform(Rule):
    _NAME = 'uniform'

    def __init__(self, dofs=None, mean=None, min=None, max=None):
        assert is_scalar(min), 'Minimum value must be a scalar'
        assert is_scalar(max), 'Maximum value must be a scalar'
        assert min < max, 'Limits are invalid, min={}, max={}'.format(min, max)

        self._min = min
        self._max = max

        if mean is not None:
            assert is_scalar(mean), 'Mean value must be a scalar'
            assert mean >= min and mean <= max, \
                'Mean value is not within the limits, mean={}, min={},' \
                ' max={}'.format(mean, min, max)
            self._mean = mean
        else:
            self._mean = (self._max - self._min) / 2 + self._min
        super(Uniform, self).__init__(dofs=dofs)

    @property
    def mean(self):
        return self._mean

    @mean.setter
    def mean(self, value):
        assert is_scalar(value), \
            'Mean value must be a scalar, received={},' \
            ' type={}'.format(value, type(value))
        self._mean = value

    @property
    def min(self):
        return self._min

    @min.setter
    def min(self, value):
        assert is_scalar(value), \
            'Min. value must be a scalar, received={},' \
            ' type={}'.format(value, type(value))
        self._min = value

    @property
    def max(self):
        return self._max

    @max.setter
    def max(self, value):
        assert is_scalar(value), \
            'Max. value must be a scalar, received={},' \
            ' type={}'.format(value, type(value))
        self._max = value

    def _get_value(self):
        return self._mean + random.uniform(
            self._min - self._mean, self._max - self._mean)

    @staticmethod
    def example():
        sample = Rule.example()
        sample['mean'] = 0
        sample['min'] = -1
        sample['max'] = 1
        sample['tag'] = Uniform._NAME
        return sample
