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
from .engine import Engine
from ..patterns import rectangular, cuboid, circular
from ...log import PCG_ROOT_LOGGER
from ...simulation import Light
from ...simulation.properties import Pose
from ...utils import is_array


class PatternEngine(Engine):
    _LABEL = 'pattern'

    _PATTERNS = ['rectangular', 'circular', 'cuboid']

    def __init__(
            self,
            assets_manager,
            constraints_manager=None,
            models=None,
            constraints=None,
            pose=[0, 0, 0, 0, 0, 0],
            pattern=None,
            collision_checker=None,
            **kwargs):
        Engine.__init__(
            self,
            assets_manager=assets_manager,
            constraints_manager=constraints_manager,
            models=models,
            constraints=constraints,
            collision_checker=collision_checker)

        if pattern not in self._PATTERNS:
            msg = 'Invalid pattern mode, options={}'.format(
                self._PATTERNS)
            PCG_ROOT_LOGGER.error(msg)
            raise ValueError(msg)

        self._pattern = pattern
        self._params = dict()
        self.set_pattern_parameters(**kwargs)

    @property
    def pattern(self):
        return self._pattern

    @pattern.setter
    def pattern(self, value):
        assert value in self._PATTERNS, \
            'Invalid pattern mode, options={}'.format(
                self._PATTERNS)
        self._pattern = value

    def set_pattern_parameters(self, **kwargs):
        if self._pattern == 'rectangular':
            assert 'x_length' in kwargs and 'y_length' in kwargs
            assert ('step_x' in kwargs and 'step_y' in kwargs) or \
                ('n_x' in kwargs and 'n_y' in kwargs)
            if 'center' in kwargs:
                assert isinstance(kwargs['center'], bool)
        elif self._pattern == 'circular':
            assert 'radius' in kwargs
            assert kwargs['radius'] > 0, 'Radius must be greater than 0'
            assert 'max_theta' in kwargs
            assert kwargs['max_theta'] >= 0 and \
                kwargs['max_theta'] <= 2 * np.pi
            assert 'step_theta' in kwargs or \
                'n_theta' in kwargs
            if 'n_radius' in kwargs:
                assert kwargs['n_radius'] > 0
            if 'step_radius' in kwargs:
                assert kwargs['step_radius'] > 0
        elif self._pattern == 'cuboid':
            assert 'x_length' in kwargs and \
                'y_length' in kwargs and \
                'z_length' in kwargs
            assert ('step_x' in kwargs and
                    'step_y' in kwargs and
                    'step_z' in kwargs) or \
                ('n_x' in kwargs and
                 'n_y' in kwargs and
                 'n_z' in kwargs)
            if 'center' in kwargs:
                assert isinstance(kwargs['center'], bool)
        if 'pose_offset' in kwargs:
            assert isinstance(kwargs['pose_offset'], Pose) or \
                is_array(kwargs['pose_offset'])
            if is_array(kwargs['pose_offset']):
                assert len(kwargs['pose_offset']) in [6, 7]
        self._params = kwargs

    def run(self):
        assert len(self._models) == 1

        models = list()

        if self._pattern == 'rectangular':
            poses = rectangular(**self._params)
        elif self._pattern == 'circular':
            poses = circular(**self._params)
        elif self._pattern == 'cuboid':
            poses = cuboid(**self._params)
        else:
            raise ValueError(
                'Invalid posioning pattern, '
                'provided={}'.format(self._pattern))

        for pose in poses:
            model = self._get_model(self._models[0])
            model.pose = pose
            # Enforce local constraints
            model = self.apply_local_constraints(model)
            models.append(model)
            self._logger.info('Adding model {}'.format(model.name))
            self._logger.info('\t {}'.format(model.pose))

        # Add models to collision checker
        for model in models:
            if not isinstance(model, Light):
                self._collision_checker.add_fixed_model(model)
                self._logger.info(
                    'Adding model <{}> as fixed model '
                    'in the collision checker'.format(
                        model.name))
        return models
