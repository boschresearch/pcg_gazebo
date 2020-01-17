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
from ..properties import Noise, Plugin
from ...parsers.sdf import create_sdf_element


class Ray(Sensor):
    def __init__(
            self,
            name='ray',
            always_on=True,
            update_rate=50,
            visualize=True,
            topic='scan',
            pose=[
                0,
                0,
                0,
                0,
                0,
                0],
            horizontal_samples=640,
            horizontal_resolution=1,
            horizontal_min_angle=0,
            horizontal_max_angle=0,
            vertical_samples=1,
            vertical_resolution=1,
            vertical_min_angle=0,
            vertical_max_angle=0,
            range_min=0,
            range_max=0,
            range_resolution=0,
            noise_mean=0,
            noise_stddev=0):
        Sensor.__init__(
            self,
            name=name,
            always_on=always_on,
            update_rate=update_rate,
            visualize=visualize,
            topic=topic,
            pose=pose)

        self._scan = dict()
        self.set_scan_properties(
            horizontal_samples=horizontal_samples,
            horizontal_resolution=horizontal_resolution,
            horizontal_min_angle=horizontal_min_angle,
            horizontal_max_angle=horizontal_max_angle,
            vertical_samples=vertical_samples,
            vertical_resolution=vertical_resolution,
            vertical_min_angle=vertical_min_angle,
            vertical_max_angle=vertical_max_angle)

        self._range = dict()
        self.set_range_properties(
            min=range_min,
            max=range_max,
            resolution=range_resolution)

        self._noise = Noise(
            mean=noise_mean,
            stddev=noise_stddev)

    def set_scan_properties(
            self,
            horizontal_samples=640,
            horizontal_resolution=1,
            horizontal_min_angle=0,
            horizontal_max_angle=0,
            vertical_samples=1,
            vertical_resolution=1,
            vertical_min_angle=0,
            vertical_max_angle=0):
        assert horizontal_min_angle <= horizontal_max_angle, \
            'Horizontal angle range is invalid'
        assert vertical_min_angle <= vertical_max_angle, \
            'Vertical angle range is invalid'
        assert horizontal_samples > 0, 'Number of horizontal ' \
            'samples must be greater than zero'
        assert vertical_samples > 0, 'Number of vertical ' \
            'samples must be greater than zero'
        self._scan['horizontal'] = dict(
            samples=horizontal_samples,
            resolution=horizontal_resolution,
            min_angle=horizontal_min_angle,
            max_angle=horizontal_max_angle
        )

        self._scan['vertical'] = dict(
            samples=vertical_samples,
            resolution=vertical_resolution,
            min_angle=vertical_min_angle,
            max_angle=vertical_max_angle
        )

    def set_range_properties(self, min=0, max=0, resolution=0):
        assert min <= max, 'Invalid range limits'
        self._range['min'] = min
        self._range['max'] = max
        self._range['resolution'] = resolution

    def set_noise(self, type='gaussian', mean=0, stddev=0):
        self._noise.type = type
        self._noise.mean = mean
        self._noise.stddev = stddev

    def add_ros_plugin(self, name='ray', frame_name='world',
                       topic_name='/scan'):
        self._plugin = Plugin()
        self._plugin.init_gazebo_ros_laser_plugin(
            name=name,
            frame_name=frame_name,
            topic_name=topic_name
        )

    def to_sdf(self):
        sensor = Sensor.to_sdf(self)
        sensor.type = 'ray'

        sensor.ray = create_sdf_element('ray')
        sensor.ray.reset(with_optional_elements=True)

        sensor.ray.scan.horizontal.resolution = \
            self._scan['horizontal']['resolution']
        sensor.ray.scan.horizontal.samples = \
            self._scan['horizontal']['samples']
        sensor.ray.scan.horizontal.min_angle = \
            self._scan['horizontal']['min_angle']
        sensor.ray.scan.horizontal.max_angle = \
            self._scan['horizontal']['max_angle']

        sensor.ray.scan.vertical.resolution = \
            self._scan['vertical']['resolution']
        sensor.ray.scan.vertical.samples = \
            self._scan['vertical']['samples']
        sensor.ray.scan.vertical.min_angle = \
            self._scan['vertical']['min_angle']
        sensor.ray.scan.vertical.max_angle = \
            self._scan['vertical']['max_angle']

        sensor.ray.range.min = self._range['min']
        sensor.ray.range.max = self._range['max']
        sensor.ray.range.resolution = self._range['resolution']

        sensor.ray.noise.type = self._noise.type
        sensor.ray.noise.mean = self._noise.mean
        sensor.ray.noise.stddev = self._noise.stddev

        return sensor

    @staticmethod
    def from_sdf(sdf):
        assert sdf.xml_element_name == 'sensor', \
            'Only SDF sensors can be parsed'
        assert sdf.type == 'ray', 'Only ray sensors can be parsed'
        sensor = Ray(name=sdf.name)

        if sdf.always_on is not None:
            sensor.always_on = sdf.always_on.value
        if sdf.visualize is not None:
            sensor.visualize = sdf.visualize.value
        if sdf.topic is not None:
            sensor.topic = sdf.topic.value
        if sdf.pose is not None:
            sensor.pose = sdf.pose.value

        if sdf.ray.scan.vertical is not None:
            vertical_samples = sdf.ray.scan.vertical.samples.value
        else:
            vertical_samples = 1

        if sdf.ray.scan.vertical is not None:
            vertical_resolution = sdf.ray.scan.vertical.resolution.value
        else:
            vertical_resolution = 1

        if sdf.ray.scan.vertical is not None:
            vertical_min_angle = sdf.ray.scan.vertical.min_angle.value
        else:
            vertical_min_angle = 0

        if sdf.ray.scan.vertical is not None:
            vertical_max_angle = sdf.ray.scan.vertical.max_angle.value
        else:
            vertical_max_angle = 0
        sensor.set_scan_properties(
            horizontal_samples=sdf.ray.scan.horizontal.samples.value,
            horizontal_resolution=sdf.ray.scan.horizontal.resolution.value,
            horizontal_min_angle=sdf.ray.scan.horizontal.min_angle.value,
            horizontal_max_angle=sdf.ray.scan.horizontal.max_angle.value,
            vertical_samples=vertical_samples,
            vertical_resolution=vertical_resolution,
            vertical_min_angle=vertical_min_angle,
            vertical_max_angle=vertical_max_angle)

        if sdf.ray.range.resolution is not None:
            sensor.set_range_properties(
                sdf.ray.range.min.value,
                sdf.ray.range.max.value,
                sdf.ray.range.resolution.value)
        else:
            sensor.set_range_properties(
                sdf.ray.range.min.value,
                sdf.ray.range.max.value,
                0)

        if sdf.ray.noise is not None:
            sensor.set_noise(
                type=sdf.ray.noise.type.value,
                mean=sdf.ray.noise.mean.value,
                stddev=sdf.ray.noise.stddev.value)

        if sdf.plugins is not None:
            assert len(
                sdf.plugins) == 1, 'Only one plugin per sensor is supported'
            sensor._plugin = Plugin.from_sdf(sdf.plugins[0])

        return sensor
