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
from .always_on import AlwaysOn
from .update_rate import UpdateRate
from .visualize import Visualize
from .topic import Topic
from .pose import Pose
from .plugin import Plugin
from .altimeter import Altimeter
from .camera import Camera
from .force_torque import ForceTorque
from .contact import Contact
from .ray import Ray
from .imu import IMU


class Sensor(XMLBase):
    _NAME = 'sensor'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        always_on=dict(creator=AlwaysOn, optional=True),
        update_rate=dict(creator=UpdateRate, optional=True),
        visualize=dict(creator=Visualize, optional=True),
        topic=dict(creator=Topic, optional=True),
        pose=dict(creator=Pose, optional=True),
        plugin=dict(creator=Plugin, n_elems='+', optional=True),
        altimeter=dict(creator=Altimeter, optional=True, mode='altimeter'),
        camera=dict(creator=Camera, optional=True, mode='camera'),
        force_torque=dict(
            creator=ForceTorque, optional=True, mode='force_torque'),
        contact=dict(creator=Contact, default=['sensor'], mode='contact'),
        ray=dict(creator=Ray, mode='ray'),
        imu=dict(creator=IMU, mode='imu')
    )

    _ATTRIBUTES = dict(
        name='default',
        type='default'
    )

    _MODES = ['none', 'altimeter', 'camera', 'contact', 'depth',
              'gps', 'gpu_ray', 'imu', 'logical_camera', 'magnetometer',
              'multicamera', 'ray', 'rfid', 'rfidtag', 'sonar',
              'wireless_receiver', 'wireless_transmitter', 'force_torque']

    def __init__(self, mode='altimeter'):
        XMLBase.__init__(self)
        self.reset(mode=mode)
        self.type = mode

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
        assert value in self._MODES, \
            'Invalid sensor type, received={}, options={}'.format(
                value, self._MODES)
        self.attributes['type'] = value
        self._mode = value

    @property
    def always_on(self):
        return self._get_child_element('always_on')

    @always_on.setter
    def always_on(self, value):
        self._add_child_element('always_on', value)

    @property
    def update_rate(self):
        return self._get_child_element('update_rate')

    @update_rate.setter
    def update_rate(self, value):
        self._add_child_element('update_rate', value)

    @property
    def visualize(self):
        return self._get_child_element('visualize')

    @visualize.setter
    def visualize(self, value):
        self._add_child_element('visualize', value)

    @property
    def topic(self):
        return self._get_child_element('topic')

    @topic.setter
    def topic(self, value):
        self._add_child_element('topic', value)

    @property
    def pose(self):
        return self._get_child_element('pose')

    @pose.setter
    def pose(self, value):
        self._add_child_element('pose', value)

    @property
    def altimeter(self):
        return self._get_child_element('altimeter')

    @altimeter.setter
    def altimeter(self, value):
        self._mode = 'altimeter'
        self._add_child_element('altimeter', value)

    @property
    def camera(self):
        return self._get_child_element('camera')

    @camera.setter
    def camera(self, value):
        self._mode = 'camera'
        self._add_child_element('camera', value)

    @property
    def force_torque(self):
        return self._get_child_element('force_torque')

    @force_torque.setter
    def force_torque(self, value):
        self._mode = 'force_torque'
        self._add_child_element('force_torque', value)

    @property
    def contact(self):
        return self._get_child_element('contact')

    @contact.setter
    def contact(self, value):
        self._mode = 'contact'
        self._add_child_element('contact', value)

    @property
    def ray(self):
        return self._get_child_element('ray')

    @ray.setter
    def ray(self, value):
        self._mode = 'ray'
        self._add_child_element('ray', value)

    @property
    def imu(self):
        return self._get_child_element('imu')

    @imu.setter
    def imu(self, value):
        self._mode = 'imu'
        self._add_child_element('imu', value)

    @property
    def plugins(self):
        return self._get_child_element('plugin')

    def reset(self, mode=None, with_optional_elements=False):
        if mode is not None:
            assert mode in self._MODES, \
                'Invalid sensor type, received={}, options={}'.format(
                    mode, self._MODES)
            self.attributes['type'] = mode
        XMLBase.reset(self, mode, with_optional_elements)

    def add_plugin(self, name, plugin=None):
        if self.plugins is not None:
            for elem in self.plugins:
                if elem.name == name:
                    print(
                        'Plugin element with name {}'
                        ' already exists'.format(name))
                    return
        if plugin is not None:
            self._add_child_element('plugin', plugin)
        else:
            plugin = Plugin()
            self._add_child_element('plugin', plugin)
        self.children['plugin'][-1].name = name

    def get_plugin_by_name(self, name):
        if self.plugins is None:
            return None
        else:
            for elem in self.plugins:
                if elem.name == name:
                    return elem
        return None
