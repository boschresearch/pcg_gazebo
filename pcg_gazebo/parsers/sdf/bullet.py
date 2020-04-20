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
from .solver import Solver
from .constraints import Constraints
from .fdir1 import FDir1
from .friction2 import Friction2
from .rolling_friction import RollingFriction
from .soft_cfm import SoftCFM
from .soft_erp import SoftERP
from .kp import Kp
from .kd import Kd
from .split_impulse import SplitImpulse
from .split_impulse_penetration_threshold import \
    SplitImpulsePenetrationThreshold


class Bullet(XMLBase):
    _NAME = 'bullet'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        solver=dict(
            creator=Solver, default=['bullet'], mode='physics'),
        constraints=dict(
            creator=Constraints, default=['bullet'], mode='physics'),
        friction=dict(
            creator=None,
            default=['scalar', 1, 0], mode='collision'),
        friction2=dict(
            creator=Friction2, default=[1], mode='collision'),
        rolling_friction=dict(
            creator=RollingFriction, default=[1], mode='collision'),
        fdir1=dict(
            creator=FDir1, default=[[0, 0, 0]], mode='collision'),
        soft_cfm=dict(
            creator=SoftCFM, default=[0], mode='contact', optional=True),
        soft_erp=dict(
            creator=SoftERP, default=[0.2], mode='contact', optional=True),
        kp=dict(
            creator=Kp, default=[1e12], mode='contact', optional=True),
        kd=dict(
            creator=Kd, default=[1], mode='contact', optional=True),
        split_impulse=dict(
            creator=SplitImpulse, default=[True], mode='contact'),
        split_impulse_penetration_threshold=dict(
            creator=SplitImpulsePenetrationThreshold,
            default=[-0.01],
            mode='contact')
    )

    _MODES = ['physics', 'collision', 'contact']

    def __init__(self, mode='physics'):
        # Solve circular dependency
        from .friction import Friction
        self._CHILDREN_CREATORS['friction']['creator'] = Friction
        super(Bullet, self).__init__()
        self.reset(mode)

    @property
    def solver(self):
        assert self._mode == 'physics', 'Solver only available for ' \
            'physics mode'
        return self._get_child_element('solver')

    @solver.setter
    def solver(self, value):
        assert self._mode == 'physics', 'Solver only available for ' \
            'physics mode'
        self._add_child_element('solver', value)

    @property
    def constraints(self):
        assert self._mode == 'physics', 'Constraints only available for ' \
            'physics mode'
        return self._get_child_element('constraints')

    @constraints.setter
    def constraints(self, value):
        assert self._mode == 'physics', 'Constraints only available for ' \
            'physics mode'
        self._add_child_element('constraints', value)

    @property
    def friction(self):
        assert self._mode == 'collision', 'Friction only available for ' \
            'collision mode'
        return self._get_child_element('friction')

    @friction.setter
    def friction(self, value):
        assert self._mode == 'collision', 'Friction only available for ' \
            'collision mode'
        self._add_child_element('friction', value)

    @property
    def friction2(self):
        assert self._mode == 'collision', 'Friction2 only available for ' \
            'collision mode'
        return self._get_child_element('friction2')

    @friction2.setter
    def friction2(self, value):
        assert self._mode == 'collision', 'Friction2 only available for ' \
            'collision mode'
        self._add_child_element('friction2', value)

    @property
    def fdir1(self):
        assert self._mode == 'collision', 'FDir1 only available for ' \
            'collision mode'
        return self._get_child_element('fdir1')

    @fdir1.setter
    def fdir1(self, value):
        assert self._mode == 'collision', 'FDir1 only available for ' \
            'collision mode'
        self._add_child_element('fdir1', value)

    @property
    def rolling_friction(self):
        assert self._mode == 'collision', 'Rolling friction only available' \
            ' for collision mode'
        return self._get_child_element('rolling_friction')

    @rolling_friction.setter
    def rolling_friction(self, value):
        assert self._mode == 'collision', 'Rolling friction only available' \
            ' for collision mode'
        self._add_child_element('rolling_friction', value)

    @property
    def split_impulse(self):
        assert self._mode == 'contact', 'Split impulse only available' \
            ' for contact mode'
        return self._get_child_element('split_impulse')

    @split_impulse.setter
    def split_impulse(self, value):
        assert self._mode == 'contact', 'Split impulse only available' \
            ' for contact mode'
        self._add_child_element('split_impulse', value)

    @property
    def split_impulse_penetration_threshold(self):
        assert self._mode == 'contact', 'Split impulse penetration threshold' \
            '  only available for contact mode'
        return self._get_child_element('split_impulse_penetration_threshold')

    @split_impulse_penetration_threshold.setter
    def split_impulse_penetration_threshold(self, value):
        assert self._mode == 'contact', 'Split impulse penetration threshold' \
            '  only available for contact mode'
        self._add_child_element('split_impulse_penetration_threshold', value)

    @property
    def soft_cfm(self):
        assert self._mode == 'contact', 'SoftCFM only available for ' \
            'contact mode'
        return self._get_child_element('soft_cfm')

    @soft_cfm.setter
    def soft_cfm(self, value):
        assert self._mode == 'contact', 'SoftCFM only available for ' \
            'contact mode'
        self._add_child_element('soft_cfm', value)

    @property
    def soft_erp(self):
        assert self._mode == 'contact', 'SoftERP only available for ' \
            'contact mode'
        return self._get_child_element('soft_erp')

    @soft_erp.setter
    def soft_erp(self, value):
        assert self._mode == 'contact', 'SoftERP only available for ' \
            'contact mode'
        self._add_child_element('soft_erp', value)

    @property
    def kp(self):
        assert self._mode == 'contact', 'Kp only available for ' \
            'contact mode'
        return self._get_child_element('kp')

    @kp.setter
    def kp(self, value):
        assert self._mode == 'contact', 'Kp only available for ' \
            'contact mode'
        self._add_child_element('kp', value)

    @property
    def kd(self):
        assert self._mode == 'contact', 'Kd only available for ' \
            'contact mode'
        return self._get_child_element('kd')

    @kd.setter
    def kd(self, value):
        assert self._mode == 'contact', 'Kd only available for ' \
            'contact mode'
        self._add_child_element('kd', value)
