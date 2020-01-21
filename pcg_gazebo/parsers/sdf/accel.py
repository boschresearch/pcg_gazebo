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
from .mean import Mean
from .stddev import StdDev
from .bias_mean import BiasMean
from .bias_stddev import BiasStdDev


class Accel(XMLBase):
    '''Noise parameters for linear accelerations.

    > *Child elements*

    * `mean`
    * `stddev`
    * `bias_mean`
    * `bias_stddev`

    > SDF versions

    * `1.4`

    > *Source*

    * [`<accel>` (SDF 1.4)](http://sdformat.org/spec?elem=sensor&ver=1.4)
    '''
    _NAME = 'accel'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        mean=dict(creator=Mean, optional=True),
        stddev=dict(creator=StdDev, optional=True),
        bias_mean=dict(creator=BiasMean, optional=True),
        bias_stddev=dict(creator=BiasStdDev, optional=True)
    )

    def __init__(self):
        '''Class constructor'''
        XMLBase.__init__(self)
        self.reset()

    @property
    def mean(self):
        '''Return the mean value SDF element, to read the value use
        `obj.mean.value`
        '''
        return self._get_child_element('mean')

    @mean.setter
    def mean(self, value):
        '''Set the mean value.

        > Input parameters

        * `value` (type: `Mean` or `float`)
        '''
        self._add_child_element('mean', value)

    @property
    def stddev(self):
        '''Return the standard deviation value SDF element,
        to read the value use `obj.stddev.value`
        '''
        return self._get_child_element('stddev')

    @stddev.setter
    def stddev(self, value):
        '''Set the standard deviation value.

        > Input parameters

        * `value` (type: `StdDev` or `float`)
        '''
        self._add_child_element('stddev', value)

    @property
    def bias_mean(self):
        '''Return the bias mean value SDF element,
        to read the value use `obj.bias_mean.value`
        '''
        return self._get_child_element('bias_mean')

    @bias_mean.setter
    def bias_mean(self, value):
        '''Set the bias mean value.

        > Input parameters

        * `value` (type: `BiasMean` or `float`)
        '''
        self._add_child_element('bias_mean', value)

    @property
    def bias_stddev(self):
        '''Return the bias standard deviation value SDF element,
        to read the value use `obj.bias_stddev.value`
        '''
        return self._get_child_element('bias_stddev')

    @bias_stddev.setter
    def bias_stddev(self, value):
        '''Set the bias standard deviation value.

        > Input parameters

        * `value` (type: `BiasStdDev` or `float`)
        '''
        self._add_child_element('bias_stddev', value)
