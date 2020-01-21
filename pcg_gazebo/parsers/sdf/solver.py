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
from .type import Type
from .min_step_size import MinStepSize
from .iters import Iters
from .precon_iters import PreConIters
from .sor import Sor
from .use_dynamic_moi_rescaling import UseDynamicMOIRescaling
from .friction_model import FrictionModel


class Solver(XMLBase):
    _NAME = 'solver'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        min_step_size=dict(creator=MinStepSize, default=[0.0001]),
        iters=dict(creator=Iters, default=[50]),
        sor=dict(creator=Sor, default=[1.3]),
        type=dict(creator=Type, default=['quick']),
        precon_iters=dict(creator=PreConIters, default=[0], mode='ode'),
        use_dynamic_moi_rescaling=dict(
            creator=UseDynamicMOIRescaling, default=[False], mode='ode'),
        friction_model=dict(creator=FrictionModel, mode='ode')
    )

    _MODES = ['ode', 'bullet']

    def __init__(self, engine='ode'):
        XMLBase.__init__(self)
        self.reset(mode=engine)
        if self._mode == 'ode':
            self.children['type'].value = 'quick'
        else:
            self.children['type'].value = 'sequential_impulse'

    @property
    def type(self):
        return self._get_child_element('type')

    @type.setter
    def type(self, value):
        if self._mode == 'ode':
            assert value in ['quick', 'world']
        else:
            assert value == 'sequential_impulse'
        self._add_child_element('type', value)

    @property
    def min_step_size(self):
        return self._get_child_element('min_step_size')

    @min_step_size.setter
    def min_step_size(self, value):
        self._add_child_element('min_step_size', value)

    @property
    def iters(self):
        return self._get_child_element('iters')

    @iters.setter
    def iters(self, value):
        self._add_child_element('iters', value)

    @property
    def sor(self):
        return self._get_child_element('sor')

    @sor.setter
    def sor(self, value):
        self._add_child_element('sor', value)

    @property
    def precon_iters(self):
        if self._mode == 'ode':
            return self._get_child_element('precon_iters')
        else:
            return None

    @precon_iters.setter
    def precon_iters(self, value):
        if self._mode == 'ode':
            self._add_child_element('precon_iters', value)
        else:
            raise AttributeError()

    @property
    def use_dynamic_moi_rescaling(self):
        if self._mode == 'ode':
            return self._get_child_element('use_dynamic_moi_rescaling')
        else:
            return None

    @use_dynamic_moi_rescaling.setter
    def use_dynamic_moi_rescaling(self, value):
        if self._mode == 'ode':
            return self._add_child_element('use_dynamic_moi_rescaling', value)
        else:
            raise AttributeError()

    @property
    def friction_model(self):
        if self._mode == 'ode':
            return self._get_child_element('friction_model')
        else:
            return None

    @friction_model.setter
    def friction_model(self, value):
        if self._mode == 'ode':
            self._add_child_element('friction_model', value)
        else:
            raise AttributeError()

    def is_valid(self):
        if self._mode == 'ode':
            if len(self.children) != 7:
                print('Number of child elements for ODE solver must be 7')
                return False
        else:
            if len(self.children) != 4:
                print('Number of child elements for ODE solver must be 4')
                return False
        is_valid = True
        for child in self.children.values():
            is_valid = is_valid and child.is_valid()
        return is_valid

    def reset(self, mode=None, with_optional_elements=False):
        XMLBase.reset(self, mode, with_optional_elements)
        if mode:
            if self._mode == 'ode':
                self.children['type'].value = 'quick'
            else:
                self.children['type'].value = 'sequential_impulse'
