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
from .ode import ODE
from .bullet import Bullet
from .torsional import Torsional


class Friction(XMLBase):
    """
    Configuration of the collision friction parameters.

    Args:
        default (float): Coefficient of friction

    Attributes:
        value (float): Stored coefficient of friction
    """
    _NAME = 'friction'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        ode=dict(creator=ODE, default=['collision'], optional=True),
        bullet=dict(creator=Bullet, default=['collision'], optional=True),
        torsional=dict(creator=Torsional, optional=True)
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def ode(self):
        return self._get_child_element('ode')

    @ode.setter
    def ode(self, value):
        self._add_child_element('ode', value)

    @property
    def bullet(self):
        return self._get_child_element('bullet')

    @bullet.setter
    def bullet(self, value):
        self._add_child_element('bullet', value)

    @property
    def torsional(self):
        return self._get_child_element('torsional')

    @torsional.setter
    def torsional(self, value):
        self._add_child_element('torsional', value)
