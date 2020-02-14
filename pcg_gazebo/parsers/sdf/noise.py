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
from .precision import Precision
from .rate import Rate
from .accel import Accel
from .type import Type


class Noise(XMLBase):
    _NAME = 'noise'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        type='none'
    )

    _ATTRIBUTES_VERSIONS = dict(
        type=['1.5', '1.6']
    )

    _CHILDREN_CREATORS = dict(
        mean=dict(creator=Mean, optional=True),
        stddev=dict(creator=StdDev, optional=True),
        bias_mean=dict(creator=BiasMean, optional=True),
        bias_stddev=dict(creator=BiasStdDev, optional=True),
        precision=dict(
            creator=Precision, optional=True),
        rate=dict(
            creator=Rate, optional=True, sdf_versions=['1.4', '1.5']),
        accel=dict(
            creator=Accel, optional=True, sdf_versions=['1.4', '1.5']),
        type=dict(
            creator=Type,
            default=['gaussian'],
            optional=True)
    )

    def __init__(self, type='none', use_type_as='attribute'):
        XMLBase.__init__(self)
        self.reset()
        self.type = 'none'
        self._description = dict(
            default='The properties of a sensor noise model'
        )

        self._use_type_as = use_type_as

    @property
    def type(self):
        return self._get_child_element('type')

    @type.setter
    def type(self, value):
        assert value in ['none', 'gaussian', 'gaussian_quantized']
        self.attributes['type'] = value
        self._add_child_element('type', value)

    @property
    def mean(self):
        return self._get_child_element('mean')

    @mean.setter
    def mean(self, value):
        self._add_child_element('mean', value)

    @property
    def stddev(self):
        return self._get_child_element('stddev')

    @stddev.setter
    def stddev(self, value):
        self._add_child_element('stddev', value)

    @property
    def bias_mean(self):
        return self._get_child_element('bias_mean')

    @bias_mean.setter
    def bias_mean(self, value):
        self._add_child_element('bias_mean', value)

    @property
    def bias_stddev(self):
        return self._get_child_element('bias_stddev')

    @bias_stddev.setter
    def bias_stddev(self, value):
        self._add_child_element('bias_stddev', value)

    @property
    def precision(self):
        return self._get_child_element('precision')

    @precision.setter
    def precision(self, value):
        self._add_child_element('precision', value)

    @property
    def rate(self):
        return self._get_child_element('rate')

    @rate.setter
    def rate(self, value):
        self._add_child_element('rate', value)

    @property
    def accel(self):
        return self._get_child_element('accel')

    @accel.setter
    def accel(self, value):
        self._add_child_element('accel', value)
