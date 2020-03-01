# Copyright (c) 2020 - The Procedural Generation for Gazebo authors
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
from ...utils import is_boolean
from ...simulation.properties import Pose


class Rule(object):
    _NAME = ''

    def __init__(self, dofs=None):
        self._dofs = dict(
            x=False,
            y=False,
            z=False,
            roll=False,
            pitch=False,
            yaw=False
        )

        if dofs is not None:
            self.dofs = dofs

    @property
    def name(self):
        return self._NAME

    @property
    def dofs(self):
        return self._dofs

    @dofs.setter
    def dofs(self, value):
        if isinstance(value, dict):
            self._dofs = dict(
                x=False,
                y=False,
                z=False,
                roll=False,
                pitch=False,
                yaw=False
            )

            for tag in self._dofs:
                if tag in value:
                    assert is_boolean(value[tag]), \
                        'DOF element <{}> must be a boolean, ' \
                        'received={}, type={}'.format(
                            value[tag], type(value[tag]))
                    self._dofs[tag] = value[tag]
        elif isinstance(value, list):
            for tag in value:
                assert tag in self._dofs, \
                    'Invalid DoF component, tag={}'.format(tag)
                self._dofs[tag] = True
        else:
            raise ValueError(
                'Invalid input for DoFs configuration, value={},'
                ' type={}'.format(value, type(value)))

    def _get_empty_pose(self):
        return Pose([0, 0, 0], [0, 0, 0])

    def _get_value(self):
        return 0

    def from_dict(self, **kwargs):
        if 'dofs' in kwargs:
            self.dofs = kwargs['dofs']

    def get_pose(self):
        pose = Pose([0, 0, 0], [0, 0, 0])

        for tag in ['x', 'y', 'z']:
            if self._dofs[tag]:
                setattr(pose, tag, self._get_value())

        pose.rpy = [
            0 if not self._dofs['roll'] else self._get_value(),
            0 if not self._dofs['pitch'] else self._get_value(),
            0 if not self._dofs['yaw'] else self._get_value(),
        ]
        return pose

    def has_dofs_conflict(self, other):
        assert isinstance(other, Rule), \
            'Input object is not a Rule object'
        for tag in self._dofs:
            if sum([self._dofs[tag], other.dofs[tag]]) == 2:
                return True
        return False

    @staticmethod
    def example():
        return dict(
            dofs=dict(x=0, y=0, z=0, roll=0, pitch=0, yaw=0))
