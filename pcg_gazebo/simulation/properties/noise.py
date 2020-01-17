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

from ...parsers.sdf import create_sdf_element


class Noise(object):
    _NOISE_TYPES = ['none', 'gaussian', 'gaussian_quantized']

    def __init__(self, mean=0, stddev=0, bias_mean=0, bias_stddev=0,
                 precision=0, type='none'):
        assert type in self._NOISE_TYPES, \
            'Invalid noise type, options=' + str(self._NOISE_TYPES)
        assert mean >= 0, 'Mean must be greater or equal to zero'
        assert stddev >= 0, 'Standard deviation must be greater' \
            ' or equal to zero'
        assert bias_mean >= 0, 'Bias mean must be greater or equal to zero'
        assert bias_stddev >= 0, 'Bias standard deviation must' \
            ' be greater or equal to zero'
        assert precision >= 0, 'Precision must be greater or equal to zero'

        self._type = type
        self._mean = mean
        self._stddev = stddev
        self._bias_mean = bias_mean
        self._bias_stddev = bias_stddev
        self._precision = precision

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, value):
        assert value in self._NOISE_TYPES, \
            'Invalid noise type, options=' + str(self._NOISE_TYPES)
        self._type = value

    @property
    def mean(self):
        return self._mean

    @mean.setter
    def mean(self, value):
        assert value >= 0, 'Mean must be greater or equal to zero'
        self._mean = value

    @property
    def stddev(self):
        return self._stddev

    @stddev.setter
    def stddev(self, value):
        assert value >= 0, 'Standard deviation must ' \
            'be greater or equal to zero'
        self._stddev = value

    @property
    def bias_mean(self):
        return self._bias_mean

    @bias_mean.setter
    def bias_mean(self, value):
        assert value >= 0, 'Bias mean must be greater or equal to zero'
        self._bias_mean = value

    @property
    def bias_stddev(self):
        return self._bias_stddev

    @bias_stddev.setter
    def bias_stddev(self, value):
        assert value >= 0, 'Bias standard deviation must be' \
            ' greater or equal to zero'
        self._bias_stddev = value

    @property
    def precision(self):
        return self._precision

    @precision.setter
    def precision(self, value):
        assert value >= 0, 'Precision must be greater or equal to zero'
        self._precision = value

    def to_sdf(self):
        sdf = create_sdf_element('noise')
        sdf.mean = self._mean
        sdf.stddev = self._stddev
        sdf.bias_mean = self._bias_mean
        sdf.bias_stddev = self._bias_stddev
        sdf.precision = self._precision
        return sdf

    @staticmethod
    def from_sdf(sdf):
        noise = Noise()
        noise.type = sdf.type.value
        if sdf.mean is not None:
            noise.mean = sdf.mean.value
        if sdf.stddev is not None:
            noise.stddev = sdf.stddev.value
        if sdf.bias_mean is not None:
            noise.bias_mean = sdf.bias_mean.value
        if sdf.bias_stddev is not None:
            noise.bias_stddev = sdf.bias_stddev.value
        if sdf.precision is not None:
            noise.precision = sdf.precision.value
        return noise
