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
import numpy as np
from ..simulation.properties import Pose
from ..log import PCG_ROOT_LOGGER


def circular(
        radius,
        max_theta=2 * np.pi,
        step_theta=None,
        step_radius=None,
        n_theta=None,
        n_radius=None,
        pose_offset=[0, 0, 0, 0, 0, 0]):
    poses = None

    assert radius > 0, \
        'Radius must be greater than zero, provided={}'.format(
            radius)

    assert max_theta >= 0 and max_theta <= 2 * np.pi, \
        'max_theta must be greater than zero and smaller' \
        ' than 2 * pi, provided={}'.format(max_theta)

    if step_theta is not None:
        assert step_theta > 0, \
            'n_theta must be greater than zero, provided={}'.format(
                n_theta)
        theta = np.arange(0, max_theta + step_theta, step_theta)
    elif n_theta is not None:
        assert n_theta > 0, \
            'Number of angle samples must be greater than 0, ' \
            'provided={}'.format(n_theta)
        if max_theta == 2 * np.pi:
            m = max_theta - max_theta / n_theta
        else:
            m = max_theta
        theta = np.linspace(0, m, n_theta)
    else:
        raise ValueError('No sampling method provided for theta')

    if step_radius is not None:
        if step_radius <= 0:
            msg = 'step_radius must be greater than zero, provided={}'.format(
                step_radius)
            PCG_ROOT_LOGGER.error(msg)
            return poses
        r = np.arange(step_radius, radius + step_radius, step_radius)
        r = r[np.nonzero(r > 0)[0]]
    elif n_radius is not None:
        assert n_radius > 0, \
            'n_radius must be greater than zero, provided={}'.format(
                n_radius)
        if n_radius == 1:
            r = np.array([radius])
        else:
            r = np.linspace(float(radius) / n_radius, radius, n_radius)
            r = r[np.nonzero(r > 0)[0]]
    elif radius > 0:
        r = np.array([radius])
    else:
        raise ValueError('Invalid radius input')

    tt, rr = np.meshgrid(theta, r)
    tt = tt.flatten()
    rr = rr.flatten()

    poses = list()
    if isinstance(pose_offset, Pose):
        offset = pose_offset
    else:
        offset = Pose(pos=pose_offset[0:3], rot=pose_offset[3::])

    for i in range(tt.size):
        poses.append(
            Pose(
                pos=[
                    rr[i] * np.cos(tt[i]),
                    rr[i] * np.sin(tt[i]),
                    0]))
        poses[-1] = offset + poses[-1]

    return poses


def rectangular(
        x_length=None,
        y_length=None,
        step_x=None,
        step_y=None,
        n_x=None,
        n_y=None,
        pose_offset=[0, 0, 0, 0, 0, 0],
        center=False):

    poses = None
    if None not in [x_length, y_length]:
        if None not in [step_x, step_y]:
            if step_x <= 0:
                PCG_ROOT_LOGGER.error('step_x must be greater than zero')
                return poses
            if step_y <= 0:
                PCG_ROOT_LOGGER.error('step_y must be greater than zero')
                return poses

            if step_x > x_length:
                PCG_ROOT_LOGGER.error(
                    'step_x should be equal or smaller than x_length')
                return poses
            if step_y > y_length:
                PCG_ROOT_LOGGER.error(
                    'step_y should be equal or smaller than y_length')
                return poses

            x = np.arange(0, x_length + step_x, step_x)
            y = np.arange(0, y_length + step_y, step_y)
        elif None not in [n_x, n_y]:
            if n_x <= 0:
                PCG_ROOT_LOGGER.error('n_x must be greater than zero')
                return poses
            if n_y <= 0:
                PCG_ROOT_LOGGER.error('n_y must be greater than zero')
                return poses

            x = np.linspace(0, x_length, n_x)
            y = np.linspace(0, y_length, n_y)
        else:
            PCG_ROOT_LOGGER.error(
                'No valid options where chosen to'
                ' generate the rectangular pattern')
            return poses

        if center:
            x = x - (x.max() - x.min()) / 2
            y = y - (y.max() - y.min()) / 2

        xx, yy = np.meshgrid(x, y)

        xx = xx.flatten()
        yy = yy.flatten()

        if isinstance(pose_offset, Pose):
            offset = pose_offset
        else:
            offset = Pose(pos=pose_offset[0:3], rot=pose_offset[3::])

        poses = list()
        for i in range(xx.size):
            poses.append(Pose(pos=[xx[i], yy[i], 0]))
            poses[-1] = offset + poses[-1]
    else:
        raise ValueError('x_length or y_length were not provided')

    return poses


def cuboid(
        x_length=None,
        y_length=None,
        z_length=None,
        step_x=None,
        step_y=None,
        step_z=None,
        n_x=None,
        n_y=None,
        n_z=None,
        pose_offset=[0, 0, 0, 0, 0, 0],
        center=False):

    poses = None
    if None not in [x_length, y_length, z_length]:
        if None not in [step_x, step_y, step_z]:
            if step_x <= 0:
                PCG_ROOT_LOGGER.error('step_x must be greater than zero')
                return poses
            if step_y <= 0:
                PCG_ROOT_LOGGER.error('step_y must be greater than zero')
                return poses
            if step_z <= 0:
                PCG_ROOT_LOGGER.error('step_z must be greater than zero')
                return poses

            if step_x > x_length:
                PCG_ROOT_LOGGER.error(
                    'step_x should be equal or smaller than x_length')
                return poses
            if step_y > y_length:
                PCG_ROOT_LOGGER.error(
                    'step_y should be equal or smaller than y_length')
                return poses
            if step_z > z_length:
                PCG_ROOT_LOGGER.error(
                    'step_z should be equal or smaller than z_length')
                return poses

            x = np.arange(0, x_length + step_x, step_x)
            y = np.arange(0, y_length + step_y, step_y)
            z = np.arange(0, z_length + step_z, step_z)
        elif None not in [n_x, n_y, n_z]:
            if n_x <= 0:
                PCG_ROOT_LOGGER.error('n_x must be greater than zero')
                return poses
            if n_y <= 0:
                PCG_ROOT_LOGGER.error('n_y must be greater than zero')
                return poses
            if n_z <= 0:
                PCG_ROOT_LOGGER.error('n_z must be greater than zero')
                return poses

            x = np.linspace(0, x_length, n_x)
            y = np.linspace(0, y_length, n_y)
            z = np.linspace(0, z_length, n_z)
        else:
            raise ValueError(
                'No valid options where chosen to'
                ' generate the rectangular pattern')

        if center:
            x = x - (x.max() - x.min()) / 2
            y = y - (y.max() - y.min()) / 2
            z = z - (z.max() - z.min()) / 2

        xx, yy, zz = np.meshgrid(x, y, z)

        xx = xx.flatten()
        yy = yy.flatten()
        zz = zz.flatten()

        if isinstance(pose_offset, Pose):
            offset = pose_offset
        else:
            offset = Pose(pos=pose_offset[0:3], rot=pose_offset[3::])

        poses = list()
        for i in range(xx.size):
            poses.append(Pose(pos=[xx[i], yy[i], zz[i]]))
            poses[-1] = offset + poses[-1]
    else:
        raise ValueError('x_length, y_length or z_length were not provided')

    return poses
