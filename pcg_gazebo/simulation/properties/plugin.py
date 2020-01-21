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

from __future__ import print_function
from ...parsers.sdf import create_sdf_element


class Plugin(object):
    def __init__(self, name=None, filename=None):
        self._name = name
        self._filename = filename
        self._params = dict()

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        assert isinstance(value, str), 'Plugin name must be a string'
        assert len(value) > 0, 'Plugin name string cannot be empty'
        self._name = value

    @property
    def filename(self):
        return self._filename

    @filename.setter
    def filename(self, value):
        assert isinstance(value, str), 'Plugin filename must be a string'
        assert len(value) > 0, 'Plugin filename string cannot be empty'
        assert value[-3::] == '.so', 'Not a valid plugin filename'
        self._filename = value

    @property
    def params(self):
        return self._params

    @params.setter
    def params(self, value):
        assert isinstance(
            value, dict), 'Parameters must be provided as a dictionary'
        self._params = value

    @property
    def param_names(self):
        return self._params.keys()

    def add_param(self, name, value):
        self._params[name] = value

    def has_param(self, name):
        return name in self._params

    def replace_parameter_value(self, old_value, new_value):
        self._replace_value_in_dict(self._params, old_value, new_value)

    @staticmethod
    def _replace_value_in_dict(data, old_value, new_value):
        for tag in data:
            if isinstance(data[tag], dict):
                Plugin._replace_value_in_dict(data[tag], old_value, new_value)
            elif data[tag] == old_value:
                data[tag] = new_value

    def init_gazebo_ros_p3d_plugin(
            self,
            name='p3d',
            robot_namespace='',
            body_name=None,
            topic_name='/groundtruth',
            frame_name='world',
            xyz_offset=[0, 0, 0],
            rpy_offset=[0, 0, 0],
            gaussian_noise=0,
            update_rate=50):
        self.name = name
        self.filename = 'libgazebo_ros_p3d.so'

        self._params = dict()
        self._params['robotNamespace'] = robot_namespace
        self._params['topicName'] = topic_name
        self._params['bodyName'] = body_name
        self._params['frameName'] = frame_name
        self._params['xyzOffset'] = xyz_offset
        self._params['rpyOffset'] = rpy_offset
        self._params['gaussianNoise'] = gaussian_noise
        self._params['updateRate'] = update_rate

    def init_gazebo_ros_imu_sensor_plugin(
            self,
            name='imu',
            robot_namespace='',
            always_on=True,
            update_rate=50,
            body_name='',
            topic_name='',
            gaussian_noise=0.0,
            frame_name='world'):
        self.name = name
        self.filename = 'libgazebo_ros_imu_sensor.so'

        self._params = dict()
        self._params['robotNamespace'] = robot_namespace
        self._params['alwaysOn'] = always_on
        self._params['updateRateHZ'] = update_rate
        self._params['bodyName'] = body_name
        self._params['topicName'] = topic_name
        self._params['gaussianNoise'] = gaussian_noise
        self._params['frameName'] = frame_name

    def init_gazebo_ros_laser_plugin(self, name='ray', frame_name='world',
                                     topic_name='/scan'):
        self.name = name
        self.filename = 'libgazebo_ros_laser.so'

        self._params = dict()
        self._params['topicName'] = topic_name
        self._params['frameName'] = frame_name

    def init_gazebo_ros_bumper_plugin(self, name='bumper', robot_namespace='',
                                      frame_name='world', topic_name='bumper'):
        self.name = name
        self.filename = 'libgazebo_ros_bumper.so'

        self._params = dict()
        self._params['robotNamespace'] = robot_namespace
        self._params['frameName'] = frame_name
        self._params['bumperTopicName'] = topic_name

    def init_gazebo_ros_camera_plugin(
            self,
            name='camera',
            robot_namespace='',
            update_rate=0,
            camera_name='camera',
            image_topic_name='image_raw',
            camera_info_topic_name='camera_info',
            frame_name='camera_link',
            hack_baseline=0.07,
            distortion_k1=0,
            distortion_k2=0,
            distortion_k3=0,
            distortion_t1=0,
            distortion_t2=0):
        self.name = name
        self.filename = 'libgazebo_ros_camera.so'

        self._params = dict()
        self._params['robotNamespace'] = robot_namespace
        self._params['updateRate'] = update_rate
        self._params['cameraName'] = camera_name
        self._params['imageTopicName'] = image_topic_name
        self._params['cameraInfoTopicName'] = camera_info_topic_name
        self._params['frameName'] = frame_name
        self._params['hackBaseline'] = hack_baseline
        self._params['distortionK1'] = distortion_k1
        self._params['distortionK2'] = distortion_k2
        self._params['distortionK3'] = distortion_k3
        self._params['distortionT1'] = distortion_t1
        self._params['distortionT2'] = distortion_t2

    def init_openni_kinect_plugin(
            self,
            name='camera',
            update_rate=0,
            camera_name='camera',
            image_topic_name='color/image_raw',
            camera_info_topic_name='color/camera_info',
            depth_image_topic_name='depth/image_rect_raw',
            depth_image_camera_info_topic_name='depth/camera_info',
            point_cloud_topic_name='depth/points',
            frame_name='camera_link',
            baseline=0.1,
            distortion_k1=0,
            distortion_k2=0,
            distortion_k3=0,
            distortion_t1=0,
            distortion_t2=0,
            point_cloud_cutoff=0,
            always_on=True):
        self.name = name
        self.filename = 'libgazebo_ros_openni_kinect.so'

        self._params = dict()
        self._params['cameraName'] = camera_name
        self._params['alwaysOn'] = always_on
        self._params['updateRate'] = update_rate
        self._params['imageTopicName'] = image_topic_name
        self._params['cameraInfoTopicName'] = camera_info_topic_name
        self._params['depthImageTopicName'] = depth_image_topic_name
        self._params['depthImageCameraInfoTopicName'] = \
            depth_image_camera_info_topic_name
        self._params['pointCloudTopicName'] = point_cloud_topic_name
        self._params['frameName'] = frame_name,
        self._params['hackBaseline'] = baseline
        self._params['distortionK1'] = distortion_k1
        self._params['distortionK2'] = distortion_k2
        self._params['distortionK3'] = distortion_k3
        self._params['distortionT1'] = distortion_t1
        self._params['distortionT2'] = distortion_t2
        self._params['pointCloudCutoff'] = point_cloud_cutoff

    def to_sdf(self):
        sdf = create_sdf_element('plugin')
        sdf.name = self._name
        sdf.filename = self._filename
        sdf.value = self._params
        return sdf

    @staticmethod
    def from_sdf(sdf):
        plugin = Plugin()
        plugin.name = sdf.name
        plugin.filename = sdf.filename
        for tag in sdf._value:
            plugin._params[tag] = sdf._value[tag]
        return plugin
