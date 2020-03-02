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
import collections
from ...simulation.properties import Pose
from ...log import PCG_ROOT_LOGGER
from ..assets_manager import AssetsManager
from ..constraints_manager import ConstraintsManager
from ..collision_checker import SingletonCollisionChecker, \
    CollisionChecker
from ..rules import create_rule


class Engine(object):
    """Base class for model factory engines. The engines are responsible
    of computing poses for models in the world and performing collision
    checks within the scenario before the final world is generated.
    This class includes methods common to all derived engine classes.

    > *Input arguments*

    * `callback_fcn_get_constraint` (*type:* `callable`, *default:* `None`):
    Handle to a function or a lambda function that returns a
    `pcg_gazebo.constraints.Constraint` associated with a tag name.
    * `models` (*type:* `list`, *default:* `None`): List of
    models names as `str` relative to the models that the
    engine will have as assets.
    * `constraints` (*type:* `list`, *default:* `None`):
    List of local constraint configurations that will be applied on
    to the engine's model assets.
    """
    _LABEL = None

    def __init__(self, assets_manager=None, constraints_manager=None,
                 models=None, constraints=None, collision_checker=None):
        if models is None:
            self._models = list()
        else:
            assert isinstance(models, list), \
                'Input models must be a string of model names'
            self._models = models

        self._poses = dict()

        self._logger = PCG_ROOT_LOGGER

        self._fixed_pose_models = list()

        self._local_constraints = dict()

        self._rules = list()

        if constraints is not None:
            for c in constraints:
                assert 'model' in c, \
                    'No constraint model provided,' \
                    ' data={}'.format(c)
                assert 'constraint' in c, \
                    'No name for constraint element ' \
                    'provided, data={}'.format(c)
                self.add_local_constraint(c['model'], c['constraint'])
        # Add collision checker
        if isinstance(collision_checker, CollisionChecker):
            self._collision_checker = collision_checker
        else:
            self._collision_checker = SingletonCollisionChecker.get_instance(
                ignore_ground_plane=True)

        if assets_manager is not None and \
                isinstance(assets_manager, AssetsManager):
            self._assets_manager = assets_manager
        else:
            self._assets_manager = AssetsManager.get_instance()

        if constraints_manager is not None and \
                isinstance(constraints_manager, ConstraintsManager):
            self._constraints_manager = constraints_manager
        else:
            self._constraints_manager = ConstraintsManager.get_instance()

    def __str__(self):
        raise NotImplementedError()

    @property
    def label(self):
        """`str`: Engine name identifier"""
        return self._LABEL

    @property
    def models(self):
        """List of `str`: List of model name tags regarding
        the engine's model assets
        """
        return self._models

    @property
    def poses(self):
        """`dict`: List of fixed poses associated with model
        names.
        """
        return self._poses

    def _test_repeated_rule_dofs(self):
        dofs = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for item in self._rules:
            for dof in item['rule'].dofs:
                assert dof in ['x', 'y', 'z', 'roll', 'pitch', 'yaw'], \
                    'Invalid DoF for rule, dof={}'.format(dof)
                if item['rule'].dofs[dof]:
                    if dof in dofs:
                        dofs.remove(dof)
                    else:
                        raise ValueError(
                            'Randomization policies have repeated DoFs'
                            ', dof={}'.format(dof))

    def add_rule(self, models, config):
        assert isinstance(config, list)
        assert isinstance(models, list)

        for rule_config in config:
            rule = create_rule(**rule_config)
            assert rule is not None, \
                'Could not create rule with provided input ' \
                'arguments, config={}'.format(rule_config)
            self._rules.append(
                dict(
                    models=models,
                    rule=rule
                )
            )
            self._test_repeated_rule_dofs()
        return True

    def get_rules_for_model(self, model_name):
        rules = list()
        for item in self._rules:
            if model_name in item['models']:
                rules.append(item['rule'])
        return rules

    def add_local_constraint(self, model_name, constraint_name):
        """Add an association of a constraint definitions with an
        specific model tag to be taken into account when running
        the placement engine.

        > *Input arguments*

        * `model_name` (*type:* `str`): Name of the model
        * `constraint_name` (*type:* `str`): Name of the constraint definition
        """
        if model_name not in self._local_constraints:
            self._local_constraints[model_name] = list()
        self._local_constraints[model_name].append(constraint_name)

    def get_local_constraints_for_model(self, model_name):
        """Return the name of the local constraint definitions for
        a model.

        > *Input arguments*

        * `model_name` (*type:* `str`): Name of the model

        > *Returns*

        List of `str`: List of constraint definition names associated
        with the model. If the model has no constraints, am empty
        list is returned.
        """
        if model_name not in self._local_constraints:
            return list()
        return self._local_constraints[model_name]

    def apply_local_constraints(self, model):
        """Apply spatial constraints to model. This will modify the pose
        of the model if it violates the constraint.

        > *Input arguments*

        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`): Model
        instance

        > *Returns*

        `pcg_gazebo.simulation.SimulationModel`: Model with modified pose.
        """
        model_local_constraints = self.get_local_constraints_for_model(
            model.name)

        for lc_name in model_local_constraints:
            lc = self._get_constraint(lc_name)
            lc.apply_constraint(model)

        return model

    def _add_pose(self, model, pose):
        """Add a new fixed-pose relative to a model.

        > *Input arguments*

        * `model` (*type:* `str`): Name of the model
        * `pose` (*type:* `list`): Pose vector (with 6 elements
        in case in contains position and Euler angles, or 7 if
        it contains position and quaternion as `(qx, qy, qz, qw)`)
        """
        assert model in self._models, \
            'Model {} is not an asset for this engine'.format(model)
        assert isinstance(pose, collections.Iterable), \
            'Pose must be an array or a vector'
        pose = list(pose)
        assert len(pose) == 6 or len(pose) == 7, \
            'Pose vector must have 6 or 7 elements'
        if model not in self._poses:
            self._poses[model] = list()

        self._poses[model].append(Pose(pos=pose[0:3], rot=pose[3::]))

    def _get_model(self, name):
        """Return a copy of the model asset.

        > *Input arguments*

        * `param` (*type:* `data_type`, *default:* `data`):
        Parameter description

        > *Returns*

        Description of return values
        """
        if name not in self._models:
            return None

        model = self._assets_manager.get(name)
        if model is None:
            self._logger.error('Model <{}> is not a valid asset'.format(name))
            return None
        return model

    def _get_constraint(self, name):
        """Return a constraint identified by the input `name`.

        > *Input arguments*

        * `name` (*type:* `str`): Name identifier of the constraint

        > *Returns*

        `pcg_gazebo.generators.constraint.Constraint`
        or one of its derived classes.
        """
        return self._constraints_manager.get(name)

    def add_model(self, model):
        """Add a model name to the list of model assets for this engine.

        > *Input arguments*

        * `model` (*type:* `str`): Name of the model
        """
        if model not in self._models:
            self._models.append(model)

    def set_fixed_pose_models(self, models):
        """Function description

        > *Input arguments*

        * `param` (*type:* `data_type`, *default:*`data`):
        Parameter description

        > *Returns*

        Description of return values
        """
        self._fixed_pose_models = models

    def run(self):
        """This function should be implemented by the derived
        classes.
        """
        raise NotImplementedError()
