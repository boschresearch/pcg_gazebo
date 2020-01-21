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

from .sensor import Sensor
from ..properties import Plugin
from ...parsers.sdf import create_sdf_element


class Contact(Sensor):
    def __init__(self, name='contact', always_on=True, update_rate=50,
                 visualize=True, topic='contact', pose=[0, 0, 0, 0, 0, 0],
                 collision_element_name=''):
        Sensor.__init__(
            self,
            name=name,
            always_on=always_on,
            update_rate=update_rate,
            visualize=visualize,
            topic=topic,
            pose=pose)
        self._name = name
        self._collision_element_name = collision_element_name

    @property
    def collision_element_name(self):
        return self._collision_element_name

    @collision_element_name.setter
    def collision_element_name(self, value):
        assert isinstance(
            value, str), 'Collision element name must be a string'
        assert len(value) > 0, 'Collision element name string cannot be empty'
        self._collision_element_name = value

    def add_ros_plugin(
            self,
            name='bumper',
            robot_namespace='',
            topic_name='bumper',
            frame_name='world'):
        self._plugin = Plugin()
        self._plugin.init_gazebo_ros_bumper_plugin(
            name=name,
            robot_namespace=robot_namespace,
            topic_name=topic_name,
            frame_name=frame_name)

    def to_sdf(self):
        sensor = Sensor.to_sdf(self)
        sensor.type = 'contact'

        sensor.contact = create_sdf_element('contact')
        sensor.contact.reset(mode='sensor', with_optional_elements=True)

        sensor.contact.topic = self._topic
        sensor.contact.collision = self._collision_element_name

        return sensor

    @staticmethod
    def from_sdf(sdf):
        sensor = Contact(name=sdf.name)

        if sdf.always_on is not None:
            sensor.always_on = sdf.always_on.value
        if sdf.visualize is not None:
            sensor.visualize = sdf.visualize.value
        if sdf.topic is not None:
            sensor.topic = sdf.topic.value
        if sdf.pose is not None:
            sensor.pose = sdf.pose.value

        assert sdf.contact is not None, 'No contact properties for sensor'
        assert sdf.contact.collision is not None
        assert sdf.contact.topic is not None

        sensor.collision_element_name = sdf.contact.collision.value
        sensor.topic = sdf.contact.topic.value

        if sdf.plugins is not None:
            assert len(
                sdf.plugins) == 1, 'Only one plugin per sensor is supported'
            sensor._plugin = Plugin.from_sdf(sdf.plugins[0])
        return sensor
