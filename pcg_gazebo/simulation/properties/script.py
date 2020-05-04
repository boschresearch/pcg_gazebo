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
from .trajectory import Trajectory
from ...parsers.sdf import create_sdf_element, \
    is_sdf_element


class Script(object):
    def __init__(self, loop=True, delay_start=0.0, auto_start=True,
                 trajectories=None):
        self._loop = loop
        self._delay_start = delay_start
        self._auto_start = auto_start
        self._trajectories = list()

    @property
    def num_trajectories(self):
        return len(self._trajectories)

    @property
    def trajectories(self):
        return self._trajectories

    def _reset_trajectory_ids(self):
        for i in range(len(self._trajectories)):
            self._trajectories[i].id = i

    def add_trajectory(self, type='__default__', **kwargs):
        if 'trajectory' in kwargs:
            if isinstance(kwargs['trajectory'], Trajectory):
                self._trajectories.append(kwargs['trajectory'])
            elif is_sdf_element(kwargs['trajectory']):
                self._trajectories.append(
                    Trajectory.from_sdf(kwargs['trajectory']))
            else:
                raise ValueError(
                    'Invalid trajectory object, type={}'.format(
                        type(kwargs['trajectory'])))
        else:
            self._trajectories.append(
                Trajectory(type=type, **kwargs))

    def get_trajectory(self, id):
        if len(self._trajectories) == 0:
            return None
        if id < 0 or id >= self._trajectories:
            return None
        return self._trajectories[id]

    def to_sdf(self):
        self._reset_trajectory_ids()
        sdf = create_sdf_element('script', 'actor')
        sdf.loop = self._loop
        sdf.delay_start = self._delay_start
        sdf.auto_start = self._auto_start

        for item in self._trajectories:
            sdf.add_trajectory(item.to_sdf())

        return sdf

    @staticmethod
    def from_sdf(sdf):
        assert sdf.xml_element_name == 'script'
        assert sdf._mode == 'actor'

        if sdf.loop is not None:
            loop = sdf.loop.value
        else:
            loop = True

        if sdf.delay_start is not None:
            delay_start = sdf.delay_start.value
        else:
            delay_start = 0

        if sdf.auto_start is not None:
            auto_start = sdf.auto_start.value
        else:
            auto_start = True

        script = Script(
            loop=loop,
            delay_start=delay_start,
            auto_start=auto_start)

        if sdf.trajectories is not None:
            for item in sdf.trajectories:
                script.add_trajectory(trajectory=item)
        return script
