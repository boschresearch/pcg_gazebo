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
from .ode import ODE
from .simbody import Simbody
from .provide_feedback import ProvideFeedback
from .sensor import Sensor
from .urdf import URDF


class JointPhysics(XMLBase):
    _NAME = 'physics'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        simbody=dict(
            creator=Simbody, default=['joint'], optional=True),
        ode=dict(
            creator=ODE, default=['joint'], optional=True),
        provide_feedback=dict(
            creator=ProvideFeedback, default=[False], optional=True))

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def simbody(self):
        return self._get_child_element('simbody')

    @simbody.setter
    def simbody(self, value):
        self._add_child_element('simbody', value)

    @property
    def ode(self):
        return self._get_child_element('ode')

    @ode.setter
    def ode(self, value):
        self._add_child_element('ode', value)

    @property
    def provide_feedback(self):
        return self._get_child_element('provide_feedback')

    @provide_feedback.setter
    def provide_feedback(self, value):
        self._add_child_element('provide_feedback', value)


class Joint(XMLBase):
    _NAME = 'joint'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        name='joint',
        type='revolute'
    )

    _CHILDREN_CREATORS = dict(
        parent=dict(creator=Parent),
        child=dict(creator=Child),
        pose=dict(creator=Pose, optional=True),
        axis=dict(creator=Axis, optional=True),
        axis2=dict(creator=Axis2, optional=True),
        physics=dict(creator=JointPhysics, optional=True),
        sensor=dict(creator=Sensor, optional=True),
        urdf=dict(creator=URDF, default=['joint'], optional=True)
    )

    _JOINT_OPTIONS = ['revolute', 'revolute2', 'gearbox', 'prismatic',
                      'ball', 'screw', 'universal', 'fixed']

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

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
        self._add_child_element('parent', value)

    @property
    def child(self):
        return self._get_child_element('child')

    @child.setter
    def child(self, value):
        self._add_child_element('child', value)

    @property
    def urdf(self):
        return self._get_child_element('urdf')

    @urdf.setter
    def urdf(self, value):
        self._add_child_element('urdf', value)

    @property
    def pose(self):
        return self._get_child_element('pose')

    @pose.setter
    def pose(self, value):
        self._add_child_element('pose', value)

    @property
    def axis(self):
        return self._get_child_element('axis')

    @axis.setter
    def axis(self, value):
        self._add_child_element('axis', value)

    @property
    def axis2(self):
        return self._get_child_element('axis2')

    @axis2.setter
    def axis2(self, value):
        self._add_child_element('axis2', value)

    @property
    def physics(self):
        return self._get_child_element('physics')

    @physics.setter
    def physics(self, value):
        self._add_child_element('physics', value)

    @property
    def sensor(self):
        return self._get_child_element('sensor')

    @sensor.setter
    def sensor(self, value):
        self._add_child_element('sensor', value)
