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
from .engine import Engine
from ...log import PCG_ROOT_LOGGER


class PatternEngine(Engine):
    _LABEL = 'pattern'

    _MODES = ['rectangular', 'circular', 'cuboid']

    def __init__(
            self,
            assets_manager,
            callback_fcn_get_constraint=None,
            models=None,
            poses=None,
            constraints=None,
            pose=[
                0,
                0,
                0,
                0,
                0,
                0],
            mode=None,
            args=None,
            collision_checker=None):
        Engine.__init__(
            self,
            assets_manager=assets_manager,
            callback_fcn_get_constraint=callback_fcn_get_constraint,
            models=models,
            constraints=constraints,
            collision_checker=collision_checker)

        if mode not in self._MODES:
            msg = 'Invalid pattern mode, options={}'.format(self._MODES)
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)

        self._mode = mode
        self._params = dict()

    def run(self):
        pass
