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
from .parent import Parent
from .child import Child
from .pose import Pose
from .axis import Axis
from .axis2 import Axis2
from .sensor import Sensor
from .urdf import URDF
from .physics import Physics
from .angle import Angle


class Joint(XMLBase):
    _NAME = 'joint'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        name='joint',
        type='revolute'
    )

    _ATTRIBUTES_MODES = dict(
        name=['state', 'model'],
        type=['model']
    )

    _CHILDREN_CREATORS = dict(
        parent=dict(creator=Parent, mode='model'),
        child=dict(creator=Child, mode='model'),
        pose=dict(creator=Pose, optional=True, mode='model'),
        axis=dict(creator=Axis, optional=True, mode='model'),
        axis2=dict(creator=Axis2, optional=True, mode='model'),
        physics=dict(
            creator=Physics,
            default=['joint'],
            optional=True,
            mode='model'),
        sensor=dict(
            creator=Sensor,
            default=['force_torque'],
            optional=True,
            mode='model'),
        urdf=dict(
            creator=URDF,
            default=['joint'],
            optional=True,
            mode='model'),
        angle=dict(creator=Angle, n_elems='+', mode='state')
    )

    _JOINT_OPTIONS = ['revolute', 'revolute2', 'gearbox', 'prismatic',
                      'ball', 'screw', 'universal', 'fixed']

    _MODES = ['state', 'model']

    def __init__(self, mode='model'):
        XMLBase.__init__(self)
        self.reset(mode=mode)

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert isinstance(value, str), 'Name must be a string'
        assert len(value) > 0, 'Name string cannot be empty'
        self.attributes['name'] = value

    @property
    def type(self):
        return self.attributes['type']

    @type.setter
    def type(self, value):
        assert isinstance(value, str), 'Type must be a string'
        assert value in self._JOINT_OPTIONS, \
            'Invalid joint type, provided={}, options={}'.format(
                value, self._JOINT_OPTIONS)
        self.attributes['type'] = value

    @property
    def parent(self):
        return self._get_child_element('parent')

    @parent.setter
    def parent(self, value):
        if self._mode != 'model':
            self._mode = 'model'
        self._add_child_element('parent', value)

    @property
    def child(self):
        return self._get_child_element('child')

    @child.setter
    def child(self, value):
        if self._mode != 'model':
            self._mode = 'model'
        self._add_child_element('child', value)

    @property
    def urdf(self):
        return self._get_child_element('urdf')

    @urdf.setter
    def urdf(self, value):
        if self._mode != 'model':
            self._mode = 'model'
        self._add_child_element('urdf', value)

    @property
    def pose(self):
        return self._get_child_element('pose')

    @pose.setter
    def pose(self, value):
        if self._mode != 'model':
            self._mode = 'model'
        self._add_child_element('pose', value)

    @property
    def axis(self):
        return self._get_child_element('axis')

    @axis.setter
    def axis(self, value):
        if self._mode != 'model':
            self._mode = 'model'
        self._add_child_element('axis', value)

    @property
    def axis2(self):
        return self._get_child_element('axis2')

    @axis2.setter
    def axis2(self, value):
        if self._mode != 'model':
            self._mode = 'model'
        self._add_child_element('axis2', value)

    @property
    def physics(self):
        return self._get_child_element('physics')

    @physics.setter
    def physics(self, value):
        if self._mode != 'model':
            self._mode = 'model'
        self._add_child_element('physics', value)

    @property
    def sensor(self):
        return self._get_child_element('sensor')

    @sensor.setter
    def sensor(self, value):
        if self._mode != 'model':
            self._mode = 'model'
        self._add_child_element('sensor', value)

    @property
    def angles(self):
        return self._get_child_element('angle')

    def add_angle(self, name=None, angle=None):
        if self._mode != 'state':
            self._mode = 'state'
        if angle is not None:
            self._add_child_element('angle', angle)
        else:
            angle = Angle()
            self._add_child_element('angle', angle)
