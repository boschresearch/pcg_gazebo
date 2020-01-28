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
from .mu import Mu
from .mu2 import Mu2
from .fdir1 import FDir1
from .slip1 import Slip1
from .slip2 import Slip2
from .soft_cfm import SoftCFM
from .soft_erp import SoftERP
from .kp import Kp
from .kd import Kd
from .max_vel import MaxVel
from .min_depth import MinDepth
from .provide_feedback import ProvideFeedback
from .cfm import CFM
from .erp import ERP
from .limit import Limit
from .slip import Slip
from .cfm_damping import CFMDamping


class ODE(XMLBase):
    _NAME = 'ode'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        solver=dict(creator=Solver, default=['ode'], mode='physics'),
        constraints=dict(creator=Constraints, default=['ode'], mode='physics'),
        mu=dict(creator=Mu, default=[1], mode='collision'),
        mu2=dict(creator=Mu2, default=[1], mode='collision'),
        fdir1=dict(creator=FDir1, default=[[0, 0, 0]], mode='collision'),
        slip1=dict(creator=Slip1, default=[0], mode='collision'),
        slip2=dict(creator=Slip2, default=[0], mode='collision'),
        soft_cfm=dict(
            creator=SoftCFM, default=[0], mode='contact', optional=True),
        soft_erp=dict(
            creator=SoftERP, default=[0.2], mode='contact', optional=True),
        kp=dict(creator=Kp, default=[1e12], mode='contact', optional=True),
        kd=dict(creator=Kd, default=[1], mode='contact', optional=True),
        max_vel=dict(
            creator=MaxVel, default=[0.01], mode='contact', optional=True),
        min_depth=dict(
            creator=MinDepth, default=[0], mode='contact', optional=True),
        provide_feedback=dict(
            creator=ProvideFeedback,
            default=[False],
            mode='joint',
            optional=True),
        cfm=dict(creator=CFM, default=[0], mode='joint', optional=True),
        erp=dict(creator=ERP, default=[0.2], mode='joint', optional=True),
        limit=dict(
            creator=Limit, default=['joint'], mode='joint', optional=True),
        slip=dict(creator=Slip, default=[0], mode='torsional', optional=True),
        cfm_damping=dict(
            creator=CFMDamping, default=[False], optional=True, mode='joint')
    )

    _MODES = ['physics', 'collision', 'contact', 'joint', 'torsional']

    def __init__(self, mode='physics'):
        XMLBase.__init__(self)
        self.reset(mode)

    @property
    def cfm_damping(self):
        return self._get_child_element('cfm_damping')

    @cfm_damping.setter
    def cfm_damping(self, value):
        if self._mode != 'joint':
            self.reset(mode='joint')
        self._add_child_element('cfm_damping', value)

    @property
    def solver(self):
        return self._get_child_element('solver')

    @solver.setter
    def solver(self, value):
        if self._mode != 'physics':
            self.reset(mode='physics')
        self._add_child_element('solver', value)

    @property
    def constraints(self):
        return self._get_child_element('constraints')

    @constraints.setter
    def constraints(self, value):
        if self._mode != 'physics':
            self.reset(mode='physics')
        self._add_child_element('constraints', value)

    @property
    def mu(self):
        return self._get_child_element('mu')

    @mu.setter
    def mu(self, value):
        if self._mode != 'collision':
            self.reset(mode='collision')
        self._add_child_element('mu', value)

    @property
    def mu2(self):
        return self._get_child_element('mu2')

    @mu2.setter
    def mu2(self, value):
        if self._mode != 'collision':
            self.reset(mode='collision')
        self._add_child_element('mu2', value)

    @property
    def fdir1(self):
        return self._get_child_element('fdir1')

    @fdir1.setter
    def fdir1(self, value):
        if self._mode != 'collision':
            self.reset(mode='collision')
        self._add_child_element('fdir1', value)

    @property
    def slip(self):
        return self._get_child_element('slip')

    @slip.setter
    def slip(self, value):
        if self._mode != 'torsional':
            self.reset(mode='torsional')
        self._add_child_element('slip', value)

    @property
    def slip1(self):
        return self._get_child_element('slip1')

    @slip1.setter
    def slip1(self, value):
        if self._mode != 'collision':
            self.reset(mode='collision')
        self._add_child_element('slip1', value)

    @property
    def slip2(self):
        return self._get_child_element('slip2')

    @slip2.setter
    def slip2(self, value):
        if self._mode != 'collision':
            self.reset(mode='collision')
        self._add_child_element('slip2', value)

    @property
    def soft_cfm(self):
        return self._get_child_element('soft_cfm')

    @soft_cfm.setter
    def soft_cfm(self, value):
        if self._mode != 'contact':
            self.reset(mode='contact')
        self._add_child_element('soft_cfm', value)

    @property
    def soft_erp(self):
        return self._get_child_element('soft_erp')

    @soft_erp.setter
    def soft_erp(self, value):
        if self._mode != 'contact':
            self.reset(mode='contact')
        self._add_child_element('soft_erp', value)

    @property
    def kp(self):
        return self._get_child_element('kp')

    @kp.setter
    def kp(self, value):
        if self._mode != 'contact':
            self.reset(mode='contact')
        self._add_child_element('kp', value)

    @property
    def kd(self):
        return self._get_child_element('kd')

    @kd.setter
    def kd(self, value):
        if self._mode != 'contact':
            self.reset(mode='contact')
        self._add_child_element('kd', value)

    @property
    def max_vel(self):
        return self._get_child_element('max_vel')

    @max_vel.setter
    def max_vel(self, value):
        if self._mode != 'contact':
            self.reset(mode='contact')
        self._add_child_element('max_vel', value)

    @property
    def min_depth(self):
        return self._get_child_element('min_depth')

    @min_depth.setter
    def min_depth(self, value):
        if self._mode != 'contact':
            self.reset(mode='contact')
        self._add_child_element('min_depth', value)

    @property
    def provide_feedback(self):
        return self._get_child_element('provide_feedback')

    @provide_feedback.setter
    def provide_feedback(self, value):
        if self._mode != 'joint':
            self.reset(mode='joint')
        self._add_child_element('provide_feedback', value)

    @property
    def limit(self):
        return self._get_child_element('limit')

    @limit.setter
    def limit(self, value):
        if self._mode != 'joint':
            self.reset(mode='joint')
        self._add_child_element('limit', value)

    @property
    def cfm(self):
        return self._get_child_element('cfm')

    @cfm.setter
    def cfm(self, value):
        if self._mode != 'joint':
            self.reset(mode='joint')
        self._add_child_element('cfm', value)

    @property
    def erp(self):
        return self._get_child_element('erp')

    @erp.setter
    def erp(self, value):
        if self._mode != 'joint':
            self.reset(mode='joint')
        self._add_child_element('erp', value)
