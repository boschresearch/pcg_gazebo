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
import collections
from ..parsers.sdf import Light as LightSDF
from .properties import Pose
from ..log import PCG_ROOT_LOGGER


class Light(object):
    def __init__(self, name='default', type='point',
                 pose=[0, 0, 0, 0, 0, 0], cast_shadows=True,
                 inner_angle=None, outer_angle=None, falloff=None):
        self._sdf = LightSDF()
        self.name = name
        self.type = type
        self.cast_shadows = cast_shadows

        self._pose = Pose()
        self.pose = pose

        if type == 'spot':
            if None in [inner_angle, outer_angle, falloff]:
                PCG_ROOT_LOGGER.warning(
                    'Spot light settings maybe invalid,'
                    ' name={}, inner_angle={}, outer_angle={}, '
                    'falloff={}'.format(
                        self.name, inner_angle, outer_angle, falloff))
            else:
                self.set_spot(
                    inner_angle=inner_angle,
                    outer_angle=outer_angle,
                    falloff=falloff
                )

    @property
    def name(self):
        return self._sdf.name

    @name.setter
    def name(self, value):
        self._sdf.name = value

    @property
    def type(self):
        return self._sdf.type

    @type.setter
    def type(self, value):
        self._sdf.type = value

    @property
    def cast_shadows(self):
        return self._sdf.cast_shadows.value

    @cast_shadows.setter
    def cast_shadows(self, value):
        self._sdf.cast_shadows = bool(value)

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, vec):
        if isinstance(vec, Pose):
            self._pose = vec
        else:
            assert isinstance(vec, collections.Iterable), \
                'Input pose vector must be iterable, received={}'.format(vec)
            assert len(vec) == 6 or len(vec) == 7, \
                'Pose must be given as position and Euler angles (x, y, z, ' \
                'roll, pitch, yaw) or position and quaternions (x, y, z, ' \
                'qw, qx, qy, qz)'
            for item in vec:
                assert isinstance(item, float) or isinstance(item, int), \
                    'All elements in pose vector must be a float or an integer'

            self._pose = Pose(pos=vec[0:3], rot=vec[3::])

    @property
    def diffuse(self):
        return self._sdf.diffuse.value

    @diffuse.setter
    def diffuse(self, value):
        self._sdf.diffuse = value

    @property
    def specular(self):
        return self._sdf.specular.value

    @specular.setter
    def specular(self, value):
        self._sdf.specular = value

    @property
    def direction(self):
        return self._sdf.direction.value

    @direction.setter
    def direction(self, value):
        self._sdf.direction = value

    @property
    def range(self):
        return self._sdf.attenuation.range.value

    @range.setter
    def range(self, value):
        self._sdf.attenuation.range = value

    @property
    def linear(self):
        return self._sdf.attenuation.linear.value

    @linear.setter
    def linear(self, value):
        self._sdf.attenuation.linear = value

    @property
    def constant(self):
        return self._sdf.attenuation.constant.value

    @constant.setter
    def constant(self, value):
        self._sdf.attenuation.constant = value

    @property
    def quadratic(self):
        return self._sdf.attenuation.quadratic.value

    @quadratic.setter
    def quadratic(self, value):
        self._sdf.attenuation.quadratic = value

    @property
    def inner_angle(self):
        return self._sdf.spot.inner_angle.value

    @inner_angle.setter
    def inner_angle(self, value):
        if self.type != 'spot':
            msg = 'Light <{}> is not a spot light'.format(self.name)
            PCG_ROOT_LOGGER.error(msg)
            return
        self._sdf.spot.inner_angle = value

    @property
    def outer_angle(self):
        return self._sdf.spot.outer_angle.value

    @outer_angle.setter
    def outer_angle(self, value):
        if self.type != 'spot':
            msg = 'Light <{}> is not a spot light'.format(self.name)
            PCG_ROOT_LOGGER.error(msg)
            return
        self._sdf.spot.outer_angle = value

    @property
    def falloff(self):
        return self._sdf.spot.falloff.value

    @falloff.setter
    def falloff(self, value):
        if self.type != 'spot':
            msg = 'Light <{}> is not a spot light'.format(self.name)
            PCG_ROOT_LOGGER.error(msg)
            return
        self._sdf.spot.falloff = value

    def copy(self):
        return Light.from_sdf(self.to_sdf())

    def set_attenuation(self, range=10, linear=1, constant=1, quadratic=0):
        self._sdf.attenuation.range = range
        self._sdf.attenuation.linear = linear
        self._sdf.attenuation.constant = constant
        self._sdf.attenuation.quadratic = quadratic

    def set_spot(self, inner_angle=0, outer_angle=0, falloff=0):
        if self.type != 'spot':
            msg = 'Light <{}> is not a spot light'.format(self.name)
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)
        if inner_angle > outer_angle:
            msg = 'Inner angle must be smaller than greater angle'
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)
        if falloff < 0:
            msg = 'Falloff must be greater than or equal to zero'
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)
        self._sdf.spot.inner_angle = inner_angle
        self._sdf.spot.outer_angle = outer_angle
        self._sdf.spot.falloff = falloff

    def to_sdf(self):
        return self._sdf

    @staticmethod
    def from_sdf(sdf):
        from copy import deepcopy
        assert sdf._NAME == 'light', 'Only light elements can be parsed'
        light = Light()
        light._sdf = deepcopy(sdf)
        return light

    @staticmethod
    def from_dict(config):
        return Light(**config)
