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

from ..properties import Pose, Plugin
from ...parsers.sdf import create_sdf_element


class Sensor(object):
    def __init__(self, name='sensor', always_on=True, update_rate=50,
                 visualize=False, topic=None, pose=[0, 0, 0, 0, 0, 0]):
        assert isinstance(always_on, bool) or always_on in [0, 1], \
            'Always on flag must be a boolean'
        assert isinstance(visualize, bool) or always_on in [0, 1], \
            'Visualize flag must be a boolean'
        assert update_rate > 0, 'Update rate must be a positive number'
        assert isinstance(topic, str), 'Topic name must be a string'
        assert len(topic) > 0, 'Topic name cannot be an empty string'

        self._always_on = bool(always_on)
        self._update_rate = update_rate
        self._visualize = bool(visualize)
        self._topic = topic
        self._plugin = None
        self._name = name
        self._pose = Pose(pose[0:3], pose[3::])

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        assert isinstance(value, str), 'Name must be a string'
        assert len(value) > 0, 'Name cannot be an empty string'
        self._name = value

    @property
    def always_on(self):
        return self._always_on

    @always_on.setter
    def always_on(self, flag):
        assert isinstance(flag, bool) or flag in [0, 1], \
            'Always on flag must be a boolean'
        self._always_on = bool(flag)

    @property
    def pose(self):
        """`pcg_gazebo.simulation.properties.Pose`: Pose of the object"""
        return self._pose

    @pose.setter
    def pose(self, vec):
        import collections
        if isinstance(vec, Pose):
            self._pose = vec
        else:
            assert isinstance(vec, collections.Iterable), \
                'Input vector must be iterable'
            assert len(vec) == 6 or len(vec) == 7, \
                'Input vector must have either 6 or 7 elements'
            for item in vec:
                assert isinstance(item, float) or isinstance(item, int), \
                    'Each pose element must be either a float or an integer'

            self._pose = Pose(pos=vec[0:3], rot=vec[3::])

    @property
    def visualize(self):
        return self._visualize

    @visualize.setter
    def visualize(self, flag):
        assert isinstance(flag, bool) or flag in [0, 1], \
            'Visualize flag must be a boolean'
        self._visualize = bool(flag)

    @property
    def update_rate(self):
        return self._update_rate

    @update_rate.setter
    def update_rate(self, value):
        assert isinstance(value, float) or isinstance(value, int), \
            'Input update rate must be a float or an integer'
        assert value > 0, 'Update rate must be greater than zero'
        self._update_rate = value

    @property
    def topic(self):
        return self._topic

    @topic.setter
    def topic(self, value):
        assert isinstance(value, str), 'Topic name must be a string'
        assert len(value) > 0, 'Topic name cannot be an empty string'
        self._topic = value

    @property
    def plugin(self):
        return self._plugin

    def set_plugin(self, name='', filename='', plugin=None, **kwargs):
        if plugin is None:
            self._plugin = Plugin(
                name=name,
                filename=filename)
            self._plugin.params = kwargs.copy()
        else:
            self._plugin = plugin

    def to_sdf(self):
        sensor = create_sdf_element('sensor')
        sensor.always_on = self._always_on
        sensor.visualize = self._visualize

        sensor.pose = self._pose.to_sdf()

        if self._topic is not None:
            sensor.topic = self._topic
        sensor.update_rate = self._update_rate

        if self._plugin is not None:
            sensor.add_plugin(self._plugin.name, self._plugin.to_sdf())

        return sensor

    @staticmethod
    def from_sdf(sdf):
        from .contact import Contact
        from .ray import Ray
        from .imu import IMU
        from .camera import Camera

        if sdf.type == 'contact':
            sensor = Contact.from_sdf(sdf)
        elif sdf.type == 'ray':
            sensor = Ray.from_sdf(sdf)
        elif sdf.type == 'imu':
            sensor = IMU.from_sdf(sdf)
        elif sdf.type in ['camera', 'depth']:
            sensor = Camera.from_sdf(sdf)
        else:
            raise NotImplementedError(sdf.name + ' ' + sdf.type)

        if sdf.plugins is not None:
            assert len(
                sdf.plugins) == 1, 'Only one plugin per sensor is supported'
            sensor._plugin = Plugin.from_sdf(sdf.plugins[0])

        return sensor
