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
from .origin import Origin
from .parent import Parent
from .child import Child
from .axis import Axis
from .dynamics import Dynamics
from .limit import Limit
from .mimic import Mimic
from .safety_controller import SafetyController
from .gazebo import Gazebo


class Joint(XMLBase):
    _NAME = 'joint'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        name='joint',
        type='revolute'
    )

    _CHILDREN_CREATORS = dict(
        origin=dict(creator=Origin, optional=True),
        parent=dict(creator=Parent),
        child=dict(creator=Child),
        limit=dict(creator=Limit),
        axis=dict(creator=Axis, optional=True),
        dynamics=dict(creator=Dynamics, optional=True),
        mimic=dict(creator=Mimic, optional=True),
        safety_controller=dict(creator=SafetyController, optional=True),
        gazebo=dict(creator=Gazebo, optional=True, default=['joint'])
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert isinstance(value, str), 'Joint name must be a string'
        assert len(value) > 0, 'Joint name string cannot be empty'
        self.attributes['name'] = value

    @property
    def type(self):
        return self.attributes['type']

    @type.setter
    def type(self, value):
        assert isinstance(value, str), 'Joint type must be a string'
        assert value in ['revolute', 'continuous', 'prismatic', 'fixed',
                         'floating', 'planar'], 'Invalid joint type'
        self.attributes['type'] = value
        if value in ['fixed', 'floating']:
            self.rm_child('axis')
            self.log_info(
                'Removing <axis> from joint since it'
                ' is of type <{}>'.format(value))

    @property
    def origin(self):
        return self._get_child_element('origin')

    @origin.setter
    def origin(self, value):
        self._add_child_element('origin', value)

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
    def axis(self):
        return self._get_child_element('axis')

    @axis.setter
    def axis(self, value):
        if self.attributes['type'] not in ['fixed', 'floating']:
            self._add_child_element('axis', value)
        else:
            self.log_warning(
                'No <axis> element can be used for fixed'
                ' or floating joints, joint={}'.format(
                    self.attributes['name']))

    @property
    def dynamics(self):
        return self._get_child_element('dynamics')

    @dynamics.setter
    def dynamics(self, value):
        self._add_child_element('dynamics', value)

    @property
    def mimic(self):
        return self._get_child_element('mimic')

    @mimic.setter
    def mimic(self, value):
        self._add_child_element('mimic', value)

    @property
    def limit(self):
        return self._get_child_element('limit')

    @limit.setter
    def limit(self, value):
        self._add_child_element('limit', value)

    @property
    def gazebo(self):
        return self._get_child_element('gazebo')

    @gazebo.setter
    def gazebo(self, value):
        self._add_child_element('gazebo', value)

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('joint')
        obj.name = self.name
        obj.type = self.type
        obj.parent = self.parent.to_sdf()
        obj.child = self.child.to_sdf()
        if self.origin is not None:
            obj.pose = self.origin.to_sdf()
        obj.axis = create_sdf_element('axis')
        obj.axis.xyz = self.axis.to_sdf()
        obj.axis.limit = self.limit.to_sdf()
        obj.axis.dynamics = self.dynamics.to_sdf()

        return obj
