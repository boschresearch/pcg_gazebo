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
from ..types import XMLBase
from .temperature import Temperature
from .pressure import Pressure
from .temperature_gradient import TemperatureGradient


class Atmosphere(XMLBase):
    _NAME = 'atmosphere'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        type='adiabatic'
    )

    _CHILDREN_CREATORS = dict(
        temperature=dict(creator=Temperature, default=[288.15], optional=True),
        pressure=dict(creator=Pressure, default=[101325.0], optional=True),
        temperature_gradient=dict(
            creator=TemperatureGradient, default=[-0.0065], optional=True)
    )

    def __init__(self):
        super(Atmosphere, self).__init__()
        self.reset()

    @property
    def type(self):
        return self._attributes['type']

    @type.setter
    def type(self, value):
        assert self._is_string(value), 'Type must be a string'
        assert value in ['adiabatic'], 'Atmosphere type must be adiabatic'
        self._attributes['type'] = str(value)

    @property
    def temperature(self):
        return self._get_child_element('temperature')

    @temperature.setter
    def temperature(self, value):
        self._add_child_element('temperature', value)

    @property
    def pressure(self):
        return self._get_child_element('pressure')

    @pressure.setter
    def pressure(self, value):
        self._add_child_element('pressure', value)

    @property
    def temperature_gradient(self):
        return self._get_child_element('temperature_gradient')

    @temperature_gradient.setter
    def temperature_gradient(self, value):
        self._add_child_element('temperature_gradient', value)
