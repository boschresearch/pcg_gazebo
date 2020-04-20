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
from .coefficient import Coefficient
from .use_patch_radius import UsePatchRadius
from .patch_radius import PatchRadius
from .surface_radius import SurfaceRadius
from .ode import ODE


class Torsional(XMLBase):
    _NAME = 'torsional'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        coefficient=dict(creator=Coefficient, optional=True),
        use_patch_radius=dict(creator=UsePatchRadius, optional=True),
        patch_radius=dict(creator=PatchRadius, optional=True),
        surface_radius=dict(creator=SurfaceRadius, optional=True),
        ode=dict(creator=ODE, default=['torsional'], optional=True)
    )

    def __init__(self):
        super(Torsional, self).__init__()
        self.reset()

    @property
    def coefficient(self):
        return self._get_child_element('coefficient')

    @coefficient.setter
    def coefficient(self, value):
        self._add_child_element('coefficient', value)

    @property
    def use_patch_radius(self):
        return self._get_child_element('use_patch_radius')

    @use_patch_radius.setter
    def use_patch_radius(self, value):
        self._add_child_element('use_patch_radius', value)

    @property
    def patch_radius(self):
        return self._get_child_element('patch_radius')

    @patch_radius.setter
    def patch_radius(self, value):
        self._add_child_element('patch_radius', value)

    @property
    def surface_radius(self):
        return self._get_child_element('surface_radius')

    @surface_radius.setter
    def surface_radius(self, value):
        self._add_child_element('surface_radius', value)

    @property
    def ode(self):
        return self._get_child_element('ode')

    @ode.setter
    def ode(self, value):
        self._add_child_element('ode', value)
