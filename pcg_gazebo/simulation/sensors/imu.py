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


class IMU(Sensor):
    def __init__(self, name='imu', always_on=True, update_rate=50,
                 visualize=False, topic='topic', pose=[0, 0, 0, 0, 0, 0]):
        Sensor.__init__(
            self,
            name=name,
            always_on=always_on,
            update_rate=update_rate,
            visualize=visualize,
            topic=topic,
            pose=pose)

        self._elements = dict()
        self._elements['angular_velocity'] = dict(
            x=Noise(),
            y=Noise(),
            z=Noise()
        )

        self._elements['linear_acceleration'] = dict(
            x=Noise(),
            y=Noise(),
            z=Noise()
        )

        self._topic = topic

    @property
    def topic(self):
        return self._topic

    @topic.setter
    def topic(self, value):
        assert isinstance(value, str), 'Topic name must be a string'
        assert len(value) > 0, 'Topic name string cannot be empty'
        self._topic = value

    def set_noise(self, element, component, mean=0, stddev=0, bias_mean=0,
                  bias_stddev=0, precision=0, type='none'):
        assert element in self._elements, \
            'Invalid IMU element, options=' + str(self._elements.keys())

        assert component in self._elements[element], \
            'Invalid component for IMU element {}'.format(element)

        self._elements[element][component] = Noise(
            mean, stddev, bias_mean, bias_stddev, precision, type)

    def get_noise(self, element, component=None):
        if element in self._elements:
            if component in self._elements[element]:
                return self._elements[element][component]
        return None

    def add_ros_plugin(
            self,
            name='imu',
            robot_namespace='',
            always_on=True,
            update_rate=50,
            body_name='',
            topic_name='',
            gaussian_noise=0.0,
            frame_name='world'):
        self._plugin = Plugin()
        self._plugin.init_gazebo_ros_imu_sensor_plugin(
            name=name,
            robot_namespace=robot_namespace,
            always_on=always_on,
            update_rate=update_rate,
            body_name=body_name,
            topic_name=topic_name,
            gaussian_noise=gaussian_noise,
            frame_name=frame_name)

    def to_sdf(self):
        sensor = Sensor.to_sdf(self)
        sensor.type = 'imu'

        sensor.imu = create_sdf_element('imu')
        sensor.imu.topic = self.topic

        sensor.imu.angular_velocity = create_sdf_element('angular_velocity')
        sensor.imu.angular_velocity.reset(with_optional_elements=True)

        sensor.imu.angular_velocity.x.noise = \
            self._elements['angular_velocity']['x'].to_sdf()
        sensor.imu.angular_velocity.y.noise = \
            self._elements['angular_velocity']['y'].to_sdf()
        sensor.imu.angular_velocity.z.noise = \
            self._elements['angular_velocity']['z'].to_sdf()

        sensor.imu.linear_acceleration = create_sdf_element(
            'linear_acceleration')
        sensor.imu.linear_acceleration.reset(with_optional_elements=True)

        sensor.imu.linear_acceleration.x.noise = \
            self._elements['linear_acceleration']['x'].to_sdf()
        sensor.imu.linear_acceleration.y.noise = \
            self._elements['linear_acceleration']['y'].to_sdf()
        sensor.imu.linear_acceleration.z.noise = \
            self._elements['linear_acceleration']['z'].to_sdf()

        # For SDF 1.4 and 1.5, the noise element must be initialized
        sensor.imu.noise = create_sdf_element('noise')
        sensor.imu.noise.rate = create_sdf_element('rate')
        sensor.imu.noise.rate.mean = \
            self._elements['angular_velocity']['x'].mean
        sensor.imu.noise.rate.stddev = \
            self._elements['angular_velocity']['x'].stddev
        sensor.imu.noise.rate.bias_mean = \
            self._elements['angular_velocity']['x'].bias_mean
        sensor.imu.noise.rate.bias_stddev = \
            self._elements['angular_velocity']['x'].bias_stddev

        sensor.imu.noise.accel = create_sdf_element('accel')

        sensor.imu.noise.accel = create_sdf_element('accel')
        sensor.imu.noise.accel.mean = \
            self._elements['linear_acceleration']['x'].mean
        sensor.imu.noise.accel.stddev = \
            self._elements['linear_acceleration']['x'].stddev
        sensor.imu.noise.accel.bias_mean = \
            self._elements['linear_acceleration']['x'].bias_mean
        sensor.imu.noise.accel.bias_stddev = \
            self._elements['linear_acceleration']['x'].bias_stddev

        return sensor

    @staticmethod
    def from_sdf(sdf):
        sensor = IMU(name=sdf.name)

        for element in ['angular_velocity', 'linear_acceleration']:
            for component in ['x', 'y', 'z']:
                if hasattr(sdf.imu, element):
                    if hasattr(getattr(sdf.imu, element), component):
                        noise = getattr(
                            getattr(
                                sdf.imu,
                                element),
                            component).noise
                        sensor._elements[element][component] = Noise.from_sdf(
                            noise)

        if sdf.plugins is not None:
            assert len(
                sdf.plugins) == 1, 'Only one plugin per sensor is supported'
            sensor._plugin = Plugin.from_sdf(sdf.plugins[0])

        return sensor
