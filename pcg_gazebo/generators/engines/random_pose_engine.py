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
from ...simulation.properties import Pose


class RandomPoseEngine(Engine):
    """Placement engine that generates a random pose for its model
    assets respecting input local constraints, if any is provided,
    such as workspace constraint. This engine performs also a
    collision check with all models already placed in the scene (except
    for models flagged as ground plane) to ensure no models are overlapping
    each other.

    > *Input arguments*

    * `assets_manager` (*type:* `pcg_gazebo.generators.AssetsManager`)
    * `callback_fcn_get_constraint` (*type:* `callable`, *default:* `None`):
    Handle to a function or a lambda function that returns a
    `pcg_gazebo.constraints.Constraint` associated with a tag name.
    * `models` (*type:* `list`, *default:* `None`): List of
    models names as `str` relative to the models that the
    engine will have as assets.
    * `constraints` (*type:* `list`, *default:* `None`): List
    of local constraint configurations that will be applied on
    to the engine's model assets.
    * `max_num` (*type:* `dict`, *default:* `None`): Maximum
    number of instances of the model assets, the key being
    the model asset's name, and the value the maximum number.
    * `no_collision` (*type:* `bool`, *default:* `True`):
    If `True`, the model instances are only added to the
    world if there are no collisions with the already existing
    models (except for models flagged as ground plane).
    * `max_area` (*type:* `float`, *default:* `1`): Percentage
    of the allowed area to fill with the models.
    * `model_picker` (*type:* `str`, *default:* `random`):
    Strategy for picking a model from the list of assets for
    the next placement in the world. Options are `random`
    (selecting a random model from the list of assets) or `area`
    (selecting the models for the biggest to the smallest).
    * `policies` (*type:* `dict`, *default:* `None`): The rules
    for model generation associated with each degree of freedom.

    ```yaml
    policies:
        - models:
         - model_1
         - model_2
         - model_3
         config:
         - dofs:
           - x
           - y
           policy:
             name: workspace
             args: area_1     # For more information on
                              # workspaces, check the
                              # class definition for
                              # `pcg_gazebo.constraints.WorkspaceConstraint`
         - dofs:
           - z
           - roll
           - pitch
           policy:
             name: value
             args: 0
         - dofs:
           - yaw
           policy:
             name: uniform
             args:
                min: -3.141592653589793
                max: 3.141592653589793
    ```
    """

    _LABEL = 'random_pose'

    def __init__(
            self,
            assets_manager=None,
            constraints_manager=None,
            is_ground_plane=False,
            models=None,
            max_num=None,
            no_collision=True,
            max_area=1,
            constraints=None,
            policies=None,
            model_picker='random',
            collision_checker=None,
            min_distance=0.0):
        Engine.__init__(
            self,
            assets_manager=assets_manager,
            constraints_manager=constraints_manager,
            models=models,
            constraints=constraints,
            collision_checker=collision_checker,
            model_picker=model_picker,
            max_num=max_num)

        assert policies is not None, 'DoF configuration was not defined'
        assert min_distance >= 0.0, \
            'Min. distance between objects ' \
            'must be equal or greater than 0'
        self._no_collision = no_collision
        self._max_num = dict()
        self._workspace = None
        self._cached_footprints = dict()
        self._min_distance = min_distance

        for item in policies:
            assert 'models' in item, \
                'Rule does not contain list of models,' \
                ' config={}'.format(item)
            self.add_rule(**item)

        if constraints is not None:
            assert isinstance(constraints, list), \
                'Constraints input must be provided as a list'
        self._counters = dict()

        for tag in self._models:
            self._counters[tag] = 0

        self._fixed_models_footprints = None

    def __str__(self):
        msg = 'Engine: {}\n'.format(self._LABEL)
        if len(self._models) > 0:
            for name in self._models:
                msg += '\tModel: {}, Max. instances: {}\n'.format(
                    name, self._max_num[name])
        else:
            msg += '\tNo models\n'
        return msg

    def _get_random_pose(self, model_name):
        """Compute a random pose for a model respecting its placement policies
        and workspace constraints, if provided.

        > *Input arguments*

        * `model_name` (*type:* `str`): Name of the model asset

        > *Returns*

        `list`: Pose vector
        """
        pose = Pose()
        for rule in self.get_rules_for_model(model_name):
            pose = pose + rule.get_pose()

        return pose

    def reset_counter(self):
        """Reset all model counters."""
        for tag in self._counters:
            self._counters[tag] = 0

    def increase_counter(self, name):
        """Increase the counter for a model.

        > *Input arguments*

        * `name` (*type:* `str`): Model name
        """
        self._counters[name] += 1

    def get_num_models(self, name):
        """Return the current value for the model counter.

        > *Input arguments*

        * `name` (*type:* `str`): Model name

        > *Returns*

        `int`: Number of models
        """
        return self._counters[name]

    def is_model_in_workspace(self, model):
        """Verify if the model is in the allowed workspace

        > *Input arguments*

        * `footprint` (*type:* `dict` or `shapely.geometries.Polygon`):
        A `shapely` polygon or a dictionary with the values being
        the footprints for different submodels.

        > *Returns*

        `bool`: `True` if the polygon is entirely contained
        inside the workspace
        """
        for rule in self.get_rules_for_model(model.name):
            if rule.name == 'workspace':
                for mesh in model.get_meshes():
                    if not rule.workspace.contains_mesh(mesh):
                        return False
        return True

    def get_list_of_footprint_polygons(self, footprint):
        """Return the list of polygons contained in the `footprint` input.

        > *Input arguments*

        * `footprint` (*type:* `dict` or `shapely.geometries.Polygon`):
        A `shapely` polygon or a dictionary with the values being
        the footprints for different submodels.

        > *Returns*

        List of `shapely.geometry.Polygon`: List of footprint polygons
        """
        polys = list()
        if isinstance(footprint, dict):
            for tag in footprint:
                polys = polys + \
                    self.get_list_of_footprint_polygons(footprint[tag])
        else:
            polys = [footprint]
        return polys

    def has_collision(self, model):
        """Run the collision checker of the input `model`
        against the current scene of the simulation.

        > *Input arguments*

        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`):


        > *Returns*

        `bool`: `True`, if any collision is detected
        """
        has_collision = \
            self._collision_checker.check_collision_with_current_scene(
                model, self._min_distance)
        return has_collision

    def run(self):
        """Run the placement engine and generate a list of models placed
        according to the input policies and respecting spatial constraints.

        > *Returns*

        List of `pcg_gazebo.simulation.SimulationModel`
        """
        if len(self.models) == 0:
            self._logger.error('No model was provided for fixed pose engine')
            return None
        models = list()

        # Reset model counter
        self.reset_counter()
        self.model_picker.reset()

        # Reset the collision checker scenario to fixed model scenario
        self._collision_checker.reset_to_fixed_model_scenario()

        collision_counter = 0
        model_reset_counter = 0
        max_collision_per_iter = 50
        max_num_model_resets = 10

        while True:
            if collision_counter == max_collision_per_iter:
                if max_num_model_resets == model_reset_counter:
                    self._logger.info(
                        'Unable to fit all models in the provided workspace')
                    break
                self._logger.info(
                    'Reset models list, max. number of collisions was reached')
                self.reset_counter()
                self.model_picker.reset()
                self._collision_checker.reset_to_fixed_model_scenario()
                models = list()
                collision_counter = 0
                model_reset_counter += 1

            model_name = self.model_picker.get_selection()

            if model_name is None:
                self._logger.info('Maximum number of models reached')
                break
            self._logger.info('Chosen model: {}'.format(model_name))

            # Retrieve model
            model = self._get_model(model_name)
            # model.show()
            if model is None:
                self._logger.error(
                    'Cannot spawn model <{}>'.format(
                        self._models[0]))
                return None
            else:
                pose = self._get_random_pose(model_name)

                model.pose = self._get_random_pose(model_name)
                self._logger.info('Generated random pose: {}'.format(
                    model.pose))
                while not self.is_model_in_workspace(model):
                    self._logger.info(
                        'Model outside of the '
                        'workspace or in collision'
                        ' with other objects!')
                    pose = self._get_random_pose(model_name)
                    self._logger.info(
                        '\t Generated random pose: {}'.format(pose))
                    model.pose = pose
                # Enforce positioning constraints
                model = self.apply_local_constraints(model)
                if self._no_collision:
                    if self.has_collision(model):
                        self._logger.info(
                            'Collision for model {} detected, '
                            'increasing collision counter and '
                            'choosing new pose, collision '
                            'counter={}'.format(
                                model_name, collision_counter + 1))
                        collision_counter += 1
                        # Decrease the model picker counter
                        self.model_picker.counter[model.name] -= 1
                        continue
                    else:
                        collision_counter = 0

            # Increase the counter for this chosen model
            self.increase_counter(model_name)

            models.append(model)

            if not self._assets_manager.is_light(model_name):
                self._collision_checker.add_model(model)

        self._logger.info('# models:')

        for tag in self._counters:
            self._logger.info('\t - {} = {}'.format(tag, self._counters[tag]))

        # Add the models to the collision checker's fixed models list
        for model in models:
            self._collision_checker.add_fixed_model(model)

        return models
