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
from ..properties import Plugin, Noise
from ...parsers.sdf import create_sdf_element, Format


class Camera(Sensor):
    def __init__(
            self,
            name='camera',
            type='camera',
            camera_name='camera',
            always_on=True,
            update_rate=50,
            visualize=True,
            topic='camera',
            pose=[0, 0, 0, 0, 0, 0],
            noise_type='gaussian',
            noise_mean=0,
            noise_stddev=0,
            horizontal_fov=1.047,
            image_width=320,
            image_height=240,
            image_format='R8G8B8',
            clip_near=0.1,
            clip_far=100,
            distortion_k1=0,
            distortion_k2=0,
            distortion_k3=0,
            distortion_p1=0,
            distortion_p2=0,
            distortion_center=[0.5, 0.5]):
        Sensor.__init__(
            self,
            name=name,
            always_on=always_on,
            update_rate=update_rate,
            visualize=visualize,
            topic=topic,
            pose=pose)
        assert type in ['camera', 'depth'], \
            'A camera sensor can be either of type depth or camera'
        self._name = name
        self._type = type
        self._camera_name = camera_name
        # Set noise model
        self._noise = Noise(
            mean=noise_mean,
            stddev=noise_stddev,
            type=noise_type)

        # Set horizontal FOV
        self._horizontal_fov = horizontal_fov

        # Set image configuration
        assert image_format in Format._VALUE_OPTIONS, 'Invalid' \
            ' image format, options={}'.format(Format._VALUE_OPTIONS)
        self._image_format = image_format
        self._image_width = image_width
        self._image_height = image_height

        # Set distortion parameters
        assert distortion_k1 >= 0, 'Radial ' \
            'distortion coefficient k1 must be ' \
            'equal or greater than zero'
        assert distortion_k2 >= 0, 'Radial ' \
            'distortion coefficient k2 must be ' \
            'equal or greater than zero'
        assert distortion_k3 >= 0, 'Radial ' \
            'distortion coefficient k3 must be ' \
            'equal or greater than zero'
        assert distortion_p1 >= 0, 'Tangential' \
            ' distortion coefficient p1 must ' \
            'be equal or greater than zero'
        assert distortion_p2 >= 0, 'Tangential' \
            ' distortion coefficient p2 must ' \
            'be equal or greater than zero'
        assert isinstance(
            distortion_center, list), 'Distortion center must be a list'
        assert len(
            distortion_center) == 2, 'Distortion center must have two elements'
        self._distortion_k1 = distortion_k1
        self._distortion_k2 = distortion_k2
        self._distortion_k3 = distortion_k3
        self._distortion_p1 = distortion_p1
        self._distortion_p2 = distortion_p2
        self._distortion_center = distortion_center

        # Set clip properties
        assert clip_far >= 0, 'Clip far must be equal or greater than zero'
        assert clip_near >= 0, 'Clip near must be equal or greater than zero'
        assert clip_far > clip_near, 'Clip far must be greater than clip near'
        self._clip_near = clip_near
        self._clip_far = clip_far

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, value):
        assert value in ['camera', 'depth'], \
            'A camera sensor can be either of type depth or camera'
        self._type = value

    @property
    def camera_name(self):
        return self._camera_name

    @camera_name.setter
    def camera_name(self, value):
        assert isinstance(value, str), 'Camera name must be a string'
        self._camera_name = value

    @property
    def noise(self):
        return self._noise

    @noise.setter
    def noise(self, values):
        if isinstance(values, dict):
            self._noise = Noise(**values)
        elif isinstance(values, Noise):
            self._noise = values
        else:
            raise ValueError(
                'Invalid noise input, received={}'.format(str(values)))

    @property
    def horizontal_fov(self):
        return self._horizontal_fov

    @horizontal_fov.setter
    def horizontal_fov(self, value):
        assert value > 0, 'Horizontal FOV must be greater than zero'
        self._horizontal_fov = value

    @property
    def image_format(self):
        return self._image_format

    @image_format.setter
    def image_format(self, value):
        assert value in Format._VALUE_OPTIONS, 'Invalid image format, ' \
            'options={}'.format(Format._VALUE_OPTIONS)
        self._image_format = value

    @property
    def image_width(self):
        return self._image_width

    @image_width.setter
    def image_width(self, value):
        assert isinstance(value, int), 'Image width must be an integer'
        assert value > 0, 'Image width must be greater than zero'
        self._image_width = value

    @property
    def image_height(self):
        return self._image_height

    @image_height.setter
    def image_height(self, value):
        assert isinstance(value, int), 'Image height must be an integer'
        assert value > 0, 'Image height must be greater than zero'
        self._image_height = value

    @property
    def distortion_k1(self):
        return self._distortion_k1

    @distortion_k1.setter
    def distortion_k1(self, value):
        self._distortion_k1 = value

    @property
    def distortion_k2(self):
        return self._distortion_k2

    @distortion_k2.setter
    def distortion_k2(self, value):
        self._distortion_k1 = value

    @property
    def distortion_k3(self):
        return self._distortion_k3

    @distortion_k3.setter
    def distortion_k3(self, value):
        self._distortion_k3 = value

    @property
    def distortion_p1(self):
        return self._distortion_p1

    @distortion_p1.setter
    def distortion_p1(self, value):
        self._distortion_p1 = value

    @property
    def distortion_p2(self):
        return self._distortion_p2

    @distortion_p2.setter
    def distortion_p2(self, value):
        self._distortion_p2 = value

    @property
    def distortion_center(self):
        return self._distortion_center

    @distortion_center.setter
    def distortion_center(self, value):
        self._distortion_center = value

    @property
    def clip_near(self):
        return self._clip_near

    @clip_near.setter
    def clip_near(self, value):
        self._clip_near = value

    @property
    def clip_far(self):
        return self._clip_far

    @clip_far.setter
    def clip_far(self, value):
        self._clip_far = value

    def add_ros_camera_plugin(
            self,
            name='camera',
            update_rate=0,
            camera_name='camera',
            image_topic_name='image_raw',
            robot_namespace='',
            camera_info_topic_name='camera_info',
            frame_name='camera_link',
            hack_baseline=0.07):

        self._plugin = Plugin()
        self._plugin.init_gazebo_ros_camera_plugin(
            name=name,
            update_rate=update_rate,
            camera_name=camera_name,
            image_topic_name=image_topic_name,
            camera_info_topic_name=camera_info_topic_name,
            frame_name=frame_name,
            hack_baseline=hack_baseline,
            distortion_k1=self._distortion_k1,
            distortion_k2=self._distortion_k2,
            distortion_k3=self._distortion_k3,
            distortion_t1=self._distortion_p1,
            distortion_t2=self._distortion_p2)

    def add_ros_openni_kinect_plugin(
            self,
            name='camera_controller',
            update_rate=0,
            always_on=True,
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
            point_cloud_cutoff=0):

        self._plugin = Plugin()
        self._plugin.init_openni_kinect_plugin(
            name,
            update_rate,
            camera_name,
            image_topic_name,
            camera_info_topic_name,
            depth_image_topic_name,
            depth_image_camera_info_topic_name,
            point_cloud_topic_name,
            frame_name,
            baseline,
            distortion_k1,
            distortion_k2,
            distortion_k3,
            distortion_t1,
            distortion_t2,
            point_cloud_cutoff,
            always_on)

    def to_sdf(self):
        sensor = Sensor.to_sdf(self)
        sensor.type = self._type

        sensor.camera = create_sdf_element('camera', 'sensor')
        sensor.camera.reset(with_optional_elements=True)
        sensor.camera.name = self._camera_name
        sensor.camera.noise = self._noise.to_sdf()

        sensor.camera.image.format = self._image_format
        sensor.camera.image.width = self._image_width
        sensor.camera.image.height = self._image_height

        sensor.camera.clip.near = self._clip_near
        sensor.camera.clip.far = self._clip_far

        sensor.camera.distortion.k1 = self._distortion_k1
        sensor.camera.distortion.k2 = self._distortion_k2
        sensor.camera.distortion.k3 = self._distortion_k3
        sensor.camera.distortion.p1 = self._distortion_p1
        sensor.camera.distortion.p2 = self._distortion_p2
        sensor.camera.distortion.center = self._distortion_center

        return sensor

    @staticmethod
    def from_sdf(sdf):
        sensor = Camera(name=sdf.name, type=sdf.type)

        assert hasattr(sdf, 'camera'), 'No camera block available'
        assert hasattr(
            sdf.camera, 'image'), 'Camera has no image properties information'

        sensor.camera_name = sdf.camera.name

        if sdf.camera.noise is not None:
            sensor.noise = Noise.from_sdf(sdf.camera.noise)

        tags = ['format', 'width', 'height']
        for tag in tags:
            if hasattr(sdf.camera.image, tag):
                setattr(sensor, tag, getattr(sdf.camera.image, tag).value)

        if hasattr(sdf.camera, 'distortion'):
            tags = ['k1', 'k2', 'k3', 'p1', 'p2']
            for tag in tags:
                if hasattr(sdf.camera.distortion, tag):
                    setattr(
                        sensor,
                        'distortion_' + tag,
                        getattr(
                            sdf.camera.distortion,
                            tag).value)

        if sdf.camera.clip is not None:
            if sdf.camera.clip.near is not None:
                sensor.clip_near = sdf.camera.clip.near.value
            if sdf.camera.clip.far is not None:
                sensor.clip_far = sdf.camera.clip.far.value

        if sdf.plugins is not None:
            assert len(
                sdf.plugins) == 1, 'Only one plugin per sensor is supported'
            sensor._plugin = Plugin.from_sdf(sdf.plugins[0])

        return sensor
