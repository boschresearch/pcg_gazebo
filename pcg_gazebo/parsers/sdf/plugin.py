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

from ..types import XMLCustom
from ...utils import is_string


class Plugin(XMLCustom):
    _NAME = 'plugin'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        name='',
        filename=''
    )

    def __init__(self, default=dict()):
        XMLCustom.__init__(self, default)
        self._has_custom_elements = True
        self.reset()
        self._value = dict()

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert isinstance(value, str), 'Plugin name must be a string'
        assert len(value) > 0, 'Plugin name cannot be empty'
        self.attributes['name'] = value

    @property
    def filename(self):
        return self.attributes['filename']

    @filename.setter
    def filename(self, value):
        assert isinstance(value, str), 'Plugin filename must be a string'
        assert len(value) > 0, 'Plugin filename cannot be empty'
        assert '.so' in value, 'Invalid plugin filename'
        self.attributes['filename'] = value

    def _add_child_element(self, tag, value):
        assert is_string(tag), 'Input tag' \
            ' must be string or unicode, received={}, type={}'.format(
                tag, type(tag))
        self._value[tag] = value

    def from_dict(self, sdf_data, ignore_tags=list()):
        XMLCustom.from_dict(self, sdf_data)

    @staticmethod
    def gazebo_ros_control(name='gazebo_ros_control', robot_namespace='',
                           control_period=None,
                           robot_param='/robot_description',
                           robot_sim_type=None):
        obj = Plugin()
        obj.name = name
        obj.filename = 'libgazebo_ros_control.so'

        params = dict()
        params['robotNamespace'] = robot_namespace
        if control_period is not None:
            assert isinstance(control_period, float) or isinstance(
                control_period, int), 'Control period must be numeric'
            assert control_period > 0, \
                'Control period must be a positive number'
            params['controlPeriod'] = control_period
        params['robotParam'] = robot_param
        if robot_sim_type is not None:
            params['robotSimType'] = robot_sim_type
        obj.value = params
        return obj

    @staticmethod
    def gazebo_ros_bumper(name='gazebo_ros_bumper', robot_namespace='',
                          bumper_topic_name='bumper_states',
                          frame_name='world'):
        obj = Plugin()
        obj.name = name
        obj.filename = 'libgazebo_ros_bumper.so'

        params = dict()
        params['robotNamespace'] = robot_namespace
        params['bumperTopicName'] = bumper_topic_name
        params['frameName'] = frame_name
        obj.value = params
        return obj

    @staticmethod
    def gazebo_ros_ft_sensor(name='gazebo_ros_ft_sensor', robot_namespace='',
                             joint_name=None, topic_name=None,
                             gaussian_noise=0, update_rate=0):
        assert topic_name is not None, 'Topic name is missing'
        assert joint_name is not None, 'Joint name is missing'
        obj = Plugin()
        obj.name = name
        obj.filename = 'libgazebo_ros_ft_sensor.so'

        params = dict()
        params['robotNamespace'] = robot_namespace
        params['topicName'] = topic_name
        params['jointName'] = joint_name
        params['gaussianNoise'] = gaussian_noise
        params['updateRate'] = update_rate
        obj.value = params
        return obj

    @staticmethod
    def gazebo_ros_p3d(name='gazebo_ros_p3d', robot_namespace='',
                       body_name=None, topic_name=None, frame_name='world',
                       xyz_offset=[0, 0, 0], rpy_offset=[0, 0, 0],
                       gaussian_noise=0, update_rate=0):
        assert topic_name is not None, 'Topic name is missing'
        assert body_name is not None, 'Body name is missing'

        obj = Plugin()
        obj.name = name
        obj.filename = 'libgazebo_ros_p3d.so'

        params = dict()
        params['robotNamespace'] = robot_namespace
        params['topicName'] = topic_name
        params['bodyName'] = body_name
        params['frameName'] = frame_name
        params['xyzOffset'] = xyz_offset
        params['rpyOffset'] = rpy_offset
        params['gaussianNoise'] = gaussian_noise
        params['updateRate'] = update_rate
        obj.value = params
        return obj
