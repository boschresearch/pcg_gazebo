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
from ..log import PCG_ROOT_LOGGER


def circular(
    radius,
    max_theta=2 *
    np.pi,
    step_theta=None,
    step_radius=None,
    n_elems_theta=None,
    n_elems_radius=None,
    pose_offset=[
        0,
        0,
        0,
        0,
        0,
        0]):
    poses = None

    if radius <= 0:
        PCG_ROOT_LOGGER.error(
            'Radius must be greater than zero, provided={}'.format(radius))
        return poses

    if max_theta <= 0 or max_theta > 2 * np.pi:
        PCG_ROOT_LOGGER.error(
            'max_theta must be greater than zero and smaller'
            ' than 2 * pi, provided={}'.format(max_theta))
        return poses

    if step_theta is not None:
        theta = np.arange(0, max_theta + step_theta, step_theta)
    elif n_elems_theta is not None:
        if n_elems_theta <= 0:
            PCG_ROOT_LOGGER.error(
                'n_elems_theta must be greater than zero, provided={}'.format(
                    n_elems_theta))
            return poses
        theta = np.linspace(0, max_theta, n_elems_theta)
    else:
        PCG_ROOT_LOGGER.error('No input to theta sampling was provided')
        return poses

    if step_radius is not None:
        if step_radius <= 0:
            msg = 'step_radius must be greater than zero, provided={}'.format(
                step_radius)
            PCG_ROOT_LOGGER.error(msg)
            return poses
        r = np.arange(0, radius + step_radius, step_radius)
        r = r[np.nonzero(r > 0)[0]]
    elif n_elems_radius is not None:
        if n_elems_radius <= 0:
            PCG_ROOT_LOGGER.error(
                'n_elems_radius must be greater than zero, provided={}'.format(
                    n_elems_radius))
            return poses
        r = np.linspace(0, radius, n_elems_radius)
        r = r[np.nonzero(r > 0)[0]]
    else:
        r = np.array([radius])

    tt, rr = np.meshgrid(theta, r)

    tt = tt.flatten()
    rr = rr.flatten()

    poses = np.zeros((tt.size, 6))
    poses[:, 0] = rr * np.cos(tt)
    poses[:, 1] = rr * np.sin(tt)
    # TODO: Apply pose offset to all poses

    return poses


def rectangular(
        x_length=None,
        y_length=None,
        step_x=None,
        step_y=None,
        n_elems_x=None,
        n_elems_y=None,
        pose_offset=[
            0,
            0,
            0,
            0,
            0,
            0],
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
        elif None not in [n_elems_x, n_elems_y]:
            if n_elems_x <= 0:
                PCG_ROOT_LOGGER.error('n_elems_x must be greater than zero')
                return poses
            if n_elems_y <= 0:
                PCG_ROOT_LOGGER.error('n_elems_y must be greater than zero')
                return poses

            x = np.linspace(0, x_length, n_elems_x)
            y = np.linspace(0, y_length, n_elems_y)
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

        poses = np.zeros((xx.size, 6))

        poses[:, 0] = xx
        poses[:, 1] = yy
        # TODO: Apply pose offset to all poses

    else:
        PCG_ROOT_LOGGER.error('x_length or y_length were not provided')

    return poses
