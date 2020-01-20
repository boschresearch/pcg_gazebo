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

from ...parsers.sdf import Pose as PoseSDF
from ...parsers.urdf import Origin
from ...transformations import quaternion_from_euler, euler_from_quaternion, \
    quaternion_conjugate, quaternion_multiply, rotation_matrix, \
    quaternion_from_matrix, random_rotation_matrix, quaternion_matrix, \
    is_same_transform
import collections
import numpy as np
from ...log import PCG_ROOT_LOGGER


class Pose(object):
    def __init__(self, pos=[0, 0, 0], rot=None):
        assert isinstance(pos, collections.Iterable), \
            'Input vector must be iterable'
        assert len(list(pos)) == 3, \
            'Position vector must have 3 elements'
        self._pos = pos
        self._quat = np.array([0, 0, 0, 1])

        if rot is not None:
            assert isinstance(rot, collections.Iterable), \
                'Input rot vector must be iterable'
            if len(list(rot)) == 3:
                rpy = list(rot)
                assert len(rpy) == 3, 'Input rpy vector must have' \
                    ' 3 elements (roll, pitch, yaw)'
                for elem in rpy:
                    assert isinstance(
                        elem, float) or isinstance(
                        elem, int), 'Each element from rpy must' \
                        ' be either a float or an integer'
                self._quat = Pose.rpy2quat(*list(rpy))
            elif len(list(rot)) == 4:
                quat = list(rot)
                assert len(quat) == 4, 'Input quat vector must have' \
                    ' 4 elements (qx, qy, qz, qw)'
                for elem in quat:
                    assert isinstance(
                        elem, float) or isinstance(
                        elem, int), 'Each element from quaternion ' \
                        'must be either a float or an integer'
                self._quat = np.array(quat)
            else:
                msg = 'Rotation vector must have either 3' \
                    ' (rpy) or 4 (quat) elements, received={}'.format(
                        len(list(rot)))
                PCG_ROOT_LOGGER.error(msg)
                raise ValueError(msg)

        self._is_updated = True

    def __str__(self):
        msg = 'Position (x, y, z) [m]: {}, {}, {}\n'.format(*self._pos)
        msg += '\t - x: {}\n'.format(self._pos[0])
        msg += '\t - y: {}\n'.format(self._pos[1])
        msg += '\t - z: {}\n'.format(self._pos[2])
        rpy = self.rpy
        msg += 'Orientation rpy (roll, pitch, yaw) (degrees): \n'
        msg += '\t - Roll: {}\n'.format(rpy[0] * 180 / np.pi)
        msg += '\t - Pitch: {}\n'.format(rpy[1] * 180 / np.pi)
        msg += '\t - Yaw: {}\n'.format(rpy[2] * 180 / np.pi)
        return msg

    def __add__(self, pose):
        q = quaternion_multiply(self.quat, pose.quat)
        p = np.array(self.position) + np.dot(
            quaternion_matrix(self.quat)[0:3, 0:3], pose.position)
        return Pose(pos=p, rot=q)

    def __sub__(self, pose):
        q = self.get_transform(pose.quat, self.quat)

        diff_p = self.position - pose.position
        p = np.dot(quaternion_matrix(pose.quat)[0:3, 0:3].T,
                   np.dot(diff_p, quaternion_matrix(pose.quat)[0:3, 0:3]))
        return Pose(pos=p, rot=q)

    def __eq__(self, pose):
        return np.allclose(
            self.position, pose.position) and is_same_transform(
            quaternion_matrix(
                self.quat), quaternion_matrix(
                pose.quat))

    @property
    def is_updated(self):
        return self._is_updated

    @property
    def position(self):
        return np.array(self._pos)

    @position.setter
    def position(self, value):
        assert isinstance(value, collections.Iterable), \
            'Input vector must be iterable'
        assert len(list(value)) == 3, \
            'Position vector must have 3 elements'
        self._pos = list(value)

    @property
    def x(self):
        return self._pos[0]

    @x.setter
    def x(self, value):
        assert isinstance(value, float) or isinstance(value, int), \
            'Input must be a float or an integer'
        self._pos[0] = value

    @property
    def y(self):
        return self._pos[1]

    @y.setter
    def y(self, value):
        assert isinstance(value, float) or isinstance(value, int), \
            'Input must be a float or an integer'
        self._pos[1] = value

    @property
    def z(self):
        return self._pos[2]

    @z.setter
    def z(self, value):
        assert isinstance(value, float) or isinstance(value, int), \
            'Input must be a float or an integer'
        self._pos[2] = value

    @property
    def rpy(self):
        return Pose.quat2rpy(self._quat)

    @rpy.setter
    def rpy(self, value):
        assert isinstance(value, collections.Iterable), \
            'Input vector must be iterable'
        assert len(list(value)) == 3, \
            'Input vector must have 3 elements'
        self._quat = Pose.rpy2quat(*value)

    @property
    def quat(self):
        return self._quat

    @quat.setter
    def quat(self, q):
        assert isinstance(q, collections.Iterable), \
            'Input vector must be iterable'
        assert len(list(q)) == 4, \
            'Input vector must have 4 elements'
        self._quat = q

    @property
    def rotation_matrix(self):
        return quaternion_matrix(self._quat)

    @staticmethod
    def rpy2quat(roll, pitch, yaw):
        return quaternion_from_euler(roll, pitch, yaw)

    @staticmethod
    def Rx(angle):
        return rotation_matrix(angle, [1, 0, 0])[0:3, 0:3]

    @staticmethod
    def Ry(angle):
        return rotation_matrix(angle, [0, 1, 0])[0:3, 0:3]

    @staticmethod
    def Rz(angle):
        return rotation_matrix(angle, [0, 0, 1])[0:3, 0:3]

    @staticmethod
    def quat2rpy(q):
        return list(euler_from_quaternion(q))

    @staticmethod
    def get_transform(q1, q2):
        return quaternion_multiply(q2, quaternion_conjugate(q1))

    @staticmethod
    def random_position():
        return Pose(pos=np.random.random(3))

    @staticmethod
    def random_orientation():
        return Pose(
            pos=[0, 0, 0],
            rot=quaternion_from_matrix(random_rotation_matrix(
                np.random.random(3))))

    @staticmethod
    def random():
        return Pose(
            pos=np.random.random(3),
            rot=Pose.random_quaternion())

    @staticmethod
    def random_quaternion():
        return quaternion_from_matrix(random_rotation_matrix(
            np.random.random(3)))

    def reset_updated(self):
        self._is_updated = False

    def to_sdf(self):
        sdf = PoseSDF()
        sdf.value = self.position.tolist() + self.rpy
        return sdf

    @staticmethod
    def from_sdf(sdf):
        assert sdf._NAME == 'pose', 'SDF element must be a <pose> entity'
        return Pose(sdf.value[0:3], sdf.value[3::])

    @staticmethod
    def from_urdf(urdf):
        assert urdf._NAME == 'origin', \
            'URDF element must be an <origin> entity'
        return Pose(urdf.xyz, urdf.rpy)

    def to_urdf(self):
        urdf = Origin()
        urdf.xyz = self.position.tolist()
        urdf.rpy = self.rpy
        return urdf
