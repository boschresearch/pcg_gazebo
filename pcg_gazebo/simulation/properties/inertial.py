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

from ...parsers.sdf import create_sdf_element
from ...utils import is_scalar
import collections
import numpy as np
from .pose import Pose


class Inertial(object):
    def __init__(self, mass=0, ixx=0, iyy=0, izz=0, ixy=0, ixz=0, iyz=0):
        self._mass = mass
        self._pose = Pose()
        self._ixx = ixx
        self._iyy = iyy
        self._izz = izz
        self._ixy = ixy
        self._iyz = iyz
        self._ixz = ixz

    def __str__(self):
        pose = self._pose.position + self._pose.rpy
        msg = 'Mass [Kg]={}\n'.format(self._mass)
        msg += 'Pose={}\n'.format(pose)
        msg += 'I =\n'
        msg += '\tIxx={}\n'.format(self._ixx)
        msg += '\tIyy={}\n'.format(self._iyy)
        msg += '\tIzz={}\n'.format(self._izz)
        msg += '\tIxy={}\n'.format(self._ixy)
        msg += '\tIxz={}\n'.format(self._ixz)
        msg += '\tIyz={}\n'.format(self._iyz)
        return msg

    @property
    def mass(self):
        return self._mass

    @mass.setter
    def mass(self, value):
        assert value > 0, 'Mass must be greater than zero'
        self._mass = value

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, vec):
        if isinstance(vec, Pose):
            self._pose = vec
        else:
            assert isinstance(vec, collections.Iterable), \
                'Input pose vector must be iterable'
            assert len(vec) == 6 or len(vec) == 7, \
                'Pose must be given as position and Euler angles (x, y, z, ' \
                'roll, pitch, yaw) or position and quaternions (x, y, z, ' \
                'qx, qy, qz, qw)'
            for item in vec:
                assert is_scalar(item), \
                    'All elements in pose vector must be a float or an integer'

            self._pose = Pose(pos=vec[0:3], rot=vec[3::])

    @property
    def ixx(self):
        return self._ixx

    @ixx.setter
    def ixx(self, value):
        assert is_scalar(value), \
            'Input value must be a float' \
            ' or an integer, provided={}'.format(
                type(value))
        self._ixx = value

    @property
    def iyy(self):
        return self._iyy

    @iyy.setter
    def iyy(self, value):
        assert is_scalar(value), \
            'Input value must be a float' \
            ' or an integer, provided={}'.format(
                type(value))
        self._iyy = value

    @property
    def izz(self):
        return self._izz

    @izz.setter
    def izz(self, value):
        assert is_scalar(value), 'Input value' \
            ' must be a float' \
            ' or an integer, provided={}'.format(
                type(value))
        self._izz = value

    @property
    def ixy(self):
        return self._ixy

    @ixy.setter
    def ixy(self, value):
        assert is_scalar(value), \
            'Input value must be a float' \
            ' or an integer, provided={}'.format(
                type(value))
        self._ixy = value

    @property
    def ixz(self):
        return self._ixz

    @ixz.setter
    def ixz(self, value):
        assert isinstance(
            value, float) or isinstance(
            value, int), \
            'Input value must be a float' \
            ' or an integer, provided={}'.format(
                type(value))
        self._ixz = value

    @property
    def iyz(self):
        return self._iyz

    @iyz.setter
    def iyz(self, value):
        assert isinstance(
            value, float) or isinstance(
            value, int), \
            'Input value must be a float' \
            ' or an integer, provided={}'.format(
                type(value))
        self._iyz = value

    @property
    def moi(self):
        return np.array([
            [self.ixx, self.ixy, self.ixz],
            [-self.ixy, self.iyy, self.iyz],
            [-self.ixz, -self.iyz, self.izz]])

    @staticmethod
    def create_inertia(inertia_type, **kwargs):
        if inertia_type == 'solid_sphere':
            return Inertial.create_solid_sphere_inertia(**kwargs)
        elif inertia_type == 'hollow_sphere':
            return Inertial.create_hollow_sphere_inertia(**kwargs)
        elif inertia_type == 'ellipsoid':
            return Inertial.create_ellipsoid_inertia(**kwargs)
        elif inertia_type == 'cuboid':
            return Inertial.create_cuboid_inertia(**kwargs)
        elif inertia_type == 'centered_rod':
            return Inertial.create_centered_rod_inertia(**kwargs)
        elif inertia_type == 'solid_cylinder':
            return Inertial.create_solid_cylinder_inertia(**kwargs)
        elif inertia_type == 'custom':
            return Inertial(**kwargs)
        else:
            return None

    @staticmethod
    def create_solid_sphere_inertia(mass, radius):
        assert mass > 0, 'Mass must be greater than zero'
        assert radius > 0, 'Radius must be greater than zero'
        inertia = Inertial()

        fac = 2. / 5
        inertia.mass = mass
        inertia.ixx = fac * mass * radius**2
        inertia.iyy = fac * mass * radius**2
        inertia.izz = fac * mass * radius**2

        return inertia

    @staticmethod
    def create_hollow_sphere_inertia(mass, radius):
        assert mass > 0, 'Mass must be greater than zero'
        assert radius > 0, 'Radius must be greater than zero'
        inertia = Inertial()

        fac = 2. / 3
        inertia.mass = mass
        inertia.ixx = fac * mass * radius**2
        inertia.iyy = fac * mass * radius**2
        inertia.izz = fac * mass * radius**2

        return inertia

    @staticmethod
    def create_ellipsoid_inertia(
            mass,
            axis_length_x,
            axis_length_y,
            axis_length_z):
        assert mass > 0
        assert axis_length_x > 0
        assert axis_length_y > 0
        assert axis_length_z > 0
        inertia = Inertial()

        fac = 1. / 5

        inertia.mass = mass
        inertia.ixx = fac * mass * (axis_length_y**2 + axis_length_z**2)
        inertia.iyy = fac * mass * (axis_length_x**2 + axis_length_z**2)
        inertia.izz = fac * mass * (axis_length_x**2 + axis_length_y**2)

        return inertia

    @staticmethod
    def create_cuboid_inertia(mass, length_x, length_y, length_z):
        assert mass > 0
        assert length_x > 0
        assert length_y > 0
        assert length_z > 0
        inertia = Inertial()

        fac = 1. / 12

        inertia.mass = mass
        inertia.ixx = fac * mass * (length_y**2 + length_z**2)
        inertia.iyy = fac * mass * (length_x**2 + length_z**2)
        inertia.izz = fac * mass * (length_x**2 + length_y**2)

        return inertia

    @staticmethod
    def create_centered_rod_inertia(mass, length, axis):
        assert mass > 0
        assert length > 0
        assert isinstance(axis, list)
        assert len(axis) == 3
        assert sum(axis) == 1
        assert axis[0] == 1 or axis[1] == 1 or axis[2] == 1
        inertia = Inertial()

        fac = 1. / 12
        inertia.mass = mass
        inertia.ixx = axis[0] * fac * mass * length**2
        inertia.iyy = axis[1] * fac * mass * length**2
        inertia.izz = axis[2] * fac * mass * length**2

        return inertia

    @staticmethod
    def create_solid_cylinder_inertia(mass, radius, length, axis):
        assert mass > 0, 'Mass must be greater than zero'
        assert length > 0, 'Length must be greater than zero'
        assert radius > 0, 'Radius must be greater than zero'
        assert isinstance(axis, list), 'Axis vector must be provided as a list'
        assert len(axis) == 3, 'Axis vector must have three elements'
        assert sum(axis) == 1, 'Axis vector must be a unit vector'
        assert axis[0] == 1 or axis[1] == 1 or axis[2] == 1
        inertia = Inertial()

        inertia.mass = mass
        i_axis = 0.5 * mass * radius**2
        i_side = 1. / 12 * mass * (3 * radius**2 + length**2)

        inertia.ixx = i_side
        inertia.iyy = i_side
        inertia.izz = i_side

        if axis[0] == 1:
            inertia.ixx = i_axis
        elif axis[1] == 1:
            inertia.iyy = i_axis
        else:
            inertia.izz = i_axis

        return inertia

    def to_sdf(self):
        sdf = create_sdf_element('inertial')
        sdf.mass = self._mass
        sdf.pose = self._pose.to_sdf()
        sdf.inertia.ixx = self._ixx
        sdf.inertia.iyy = self._iyy
        sdf.inertia.izz = self._izz
        sdf.inertia.ixy = self._ixy
        sdf.inertia.ixz = self._ixz
        sdf.inertia.iyz = self._iyz
        return sdf

    @staticmethod
    def from_sdf(sdf):
        assert sdf._NAME == 'inertial', \
            'Input SDF element must be of type inertial'
        inertial = Inertial()
        inertial.mass = sdf.mass.value
        inertial._pose = Pose.from_sdf(sdf.pose)
        inertial.ixx = sdf.inertia.ixx.value
        inertial.iyy = sdf.inertia.iyy.value
        inertial.izz = sdf.inertia.izz.value
        inertial.ixy = sdf.inertia.ixy.value
        inertial.ixz = sdf.inertia.ixz.value
        inertial.iyz = sdf.inertia.iyz.value
        return inertial
