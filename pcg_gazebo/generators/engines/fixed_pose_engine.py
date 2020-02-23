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
from ...simulation import Light


class FixedPoseEngine(Engine):
    """Engine that just places models on pre-configured fixed poses. This
    engine only accepts one model asset.

    * `callback_fcn_get_constraint` (*type:* `callable`,
    *default:* `None`): Handle to a function or a lambda
    function that returns a `pcg_gazebo.constraints.Constraint`
    associated with a tag name.
    * `models` (*type:* `list`, *default:* `None`): List
    of models names as `str` relative to the models that
    the engine will have as assets.
    * `constraints` (*type:* `list`, *default:* `None`):
    List of local constraint configurations that will be
    applied on to the engine's model assets.
    * `poses` (*type:* `list`): List of 6- (position and
    Euler angles) or 7 element (position and quaternion) poses.
    """
    _LABEL = 'fixed_pose'

    def __init__(
            self,
            assets_manager,
            constraints_manager=None,
            models=None,
            poses=None,
            constraints=None,
            collision_checker=None):
        Engine.__init__(
            self,
            assets_manager=assets_manager,
            constraints_manager=constraints_manager,
            models=models,
            constraints=constraints,
            collision_checker=collision_checker)

        if models is not None:
            assert len(
                models) == 1, 'The fixed pose engine can use only one model'

            if poses is not None:
                assert isinstance(poses, list), 'Input poses must be a list'
                for pose in poses:
                    self.add_pose(pose)

    def __str__(self):
        msg = 'Engine: {}\n'.format(self._LABEL)
        if len(self._models) == 1:
            msg += '\tModel: {}\n'.format(self._models[0])
        else:
            msg += '\tNo models\n'
        if len(self._poses):
            msg += '\tPoses: \n'
            for pose in self._poses[self._models[0]]:
                msg += '\t\t - {}\n'.format(pose.position + pose.rpy)
        else:
            msg += '\tNo poses\n'
        return msg

    def add_pose(self, pose):
        """Add pose to the list of fixed-poses.

        > *Input arguments*

        * `pose` (*type:* `list`): 6- (position and Euler angles) or 7 element
        (position and quaternion) poses.
        """
        if len(self.models) == 0:
            self._logger.error('No model was provided for fixed pose engine')
            return False

        self._add_pose(self._models[0], pose)

    def run(self):
        """Generate instances of the model asset for all
        the poses provided. If any local constraints were also provided,
        they will be applied to the model after its placement.

        > *Returns*

        List of `pcg_gazebo.simulation.SimulationModel`: Model instances.
        """
        if len(self.models) == 0:
            self._logger.error('No model was provided for fixed pose engine')
            return None
        models = list()
        for pose in self._poses[self._models[0]]:
            model = self._get_model(self._models[0])
            if model is None:
                self._logger.error(
                    'Cannot spawn model <{}>'.format(
                        self._models[0]))
                return None

            pose = [float(x) for x in list(pose.position) + list(pose.quat)]
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
