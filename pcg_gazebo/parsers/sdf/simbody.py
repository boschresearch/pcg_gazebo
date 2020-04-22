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
from .min_step_size import MinStepSize
from .accuracy import Accuracy
from .max_transient_velocity import MaxTransientVelocity
from .contact import Contact
from .must_be_loop_joint import MustBeLoopJoint


class Simbody(XMLBase):
    _NAME = 'simbody'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        min_step_size=dict(
            creator=MinStepSize,
            default=[0.0001],
            mode='physics'),
        accuracy=dict(
            creator=Accuracy,
            default=[0.001],
            mode='physics'),
        max_transient_velocity=dict(
            creator=MaxTransientVelocity,
            default=[0.01],
            mode='physics'),
        contact=dict(
            creator=Contact,
            default=['simbody'],
            optional=True,
            mode='physics'),
        must_be_loop_joint=dict(
            creator=MustBeLoopJoint,
            default=[False],
            optional=True,
            mode='joint'))

    _MODES = ['physics', 'joint']

    def __init__(self, mode='physics'):
        super(Simbody, self).__init__()
        self.reset(mode=mode)

    @property
    def min_step_size(self):
        return self._get_child_element('min_step_size')

    @min_step_size.setter
    def min_step_size(self, value):
        if self._mode != 'physics':
            self.reset(mode='physics')
        self._add_child_element('min_step_size', value)

    @property
    def accuracy(self):
        return self._get_child_element('accuracy')

    @accuracy.setter
    def accuracy(self, value):
        if self._mode != 'physics':
            self.reset(mode='physics')
        self._add_child_element('accuracy', value)

    @property
    def max_transient_velocity(self):
        return self._get_child_element('max_transient_velocity')

    @max_transient_velocity.setter
    def max_transient_velocity(self, value):
        if self._mode != 'physics':
            self.reset(mode='physics')
        self._add_child_element('max_transient_velocity', value)

    @property
    def contact(self):
        return self._get_child_element('contact')

    @contact.setter
    def contact(self, value):
        if self._mode != 'physics':
            self.reset(mode='physics')
        self._add_child_element('contact', value)

    @property
    def must_be_loop_joint(self):
        return self._get_child_element('must_be_loop_joint')

    @must_be_loop_joint.setter
    def must_be_loop_joint(self, value):
        if self._mode != 'joint':
            self.reset(mode='joint')
        self._add_child_element('must_be_loop_joint', value)
