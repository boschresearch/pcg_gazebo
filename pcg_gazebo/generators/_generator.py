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
from ..log import PCG_ROOT_LOGGER
from ..simulation import is_gazebo_model, SimulationModel, \
    Light, ModelGroup
from .assets_manager import AssetsManager
from .engine_manager import EngineManager
from ..utils import generate_random_string, is_string, load_yaml


class _Generator(object):
    def __init__(self, name='default', **kwargs):
        self._assets = AssetsManager.get_instance()
        self._engines = EngineManager()
        self._simulation_entity = None
        self._name = None

        # Set the name provided to the simulation
        # entity to be generated
        self.name = name
        self.from_dict(kwargs)

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        assert is_string(value), \
            'Name should be a string'
        assert len(value) > 0, \
            'Name cannot be an empty string'
        self._name = value

    @property
    def assets(self):
        """List of `pcg_gazebo.simulation.SimulationModel`: List of model
        assets that will be used of the world generation.
        """
        return self._assets

    @property
    def engines(self):
        """`dict` of `pcg_gazebo.generators.engines`: Dictionary with the
        model creation engines.
        """
        return self._engines

    @property
    def constraints(self):
        """`dict` of `pcg_gazebo.generators.constraints`: Dictionary with the
        positioning constraints.
        """
        return self._engines.constraints_manager

    def _add_asset_to_simulation_entity(self, obj):
        if isinstance(obj, SimulationModel):
            PCG_ROOT_LOGGER.info(
                'Adding model {} to world'.format(
                    obj.name))
            return self._simulation_entity.add_model(obj.name, obj)
        elif isinstance(obj, ModelGroup):
            PCG_ROOT_LOGGER.info(
                'Adding model group {} to world'.format(
                    obj.name))
            return self._simulation_entity.add_model_group(obj, obj.name)
        elif isinstance(obj, Light):
            return self._simulation_entity.add_light(obj.name, obj)
        return False

    def add_engine(self, tag, engine_name, models, **kwargs):
        """Add a new model creator engine to the internal engines list.

        > *Input arguments*

        * `engine_name` (*type:* `str`): Name of the engine class
        to be created
        * `models` (*type:* list of `str`): Name of the models that
        will be assets to the created engine
        * `kwargs` (*type:* `dict`): Input arguments to the created engine.
        """
        return self._engines.add(tag, engine_name, models, **kwargs)

    def add_constraint(self, name, type, **kwargs):
        """Add a new positioning constraint class to the internal
        constraints list.

        > *Input arguments*

        * `name` (*type:* `str`): ID name for the constraint class instance
        * `type` (*type:* `str`): Name of the constraints class to be created
        * `kwargs` (*type:* `dict`): Input arguments for the constraint class
        to be created
        """
        return self._engines.add_constraint(name, type, **kwargs)

    def add_asset(self, *args, **kwargs):
        """Add a new model asset that can be used by the engines and
        added to the generated world.

        > *Input arguments*

        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`):
        Simulation model
        """
        return self._assets.add(*args, **kwargs)

    def set_model_as_ground_plane(self, model_name):
        """Flag a model asset as part of the ground plane. This procedure will
        affect the collision checks during the automatic placement of models in
        the world using the placement engines.

        > *Input arguments*

        * `model_name` (*type:* `str`): Name of the model asset
        """
        return self._assets.set_asset_as_ground_plane(model_name)

    def get_asset(self, name):
        """Return a simulation model asset.

        > *Input arguments*

        * `name` (*type:* `str`): Name of the model asset.

        > *Returns*

        The model asset as `pcg_gazebo.simulation.SimulationModel`.
        `None` if `name` cannot be found in the list of model assets.
        """
        return self._assets.get(name)

    def get_constraint(self, name):
        """Return a positioning constraint configuration.

        > *Input arguments*

        * `param` (*type:* `data_type`, *default:* `data`):
        Parameter description

        > *Returns*

        Description of return values
        """
        return self._engines.constraints_manager.get(name)

    def add_gazebo_model(self, model_name, pose=[0, 0, 0, 0, 0, 0]):
        """Add an existent Gazebo model to the world in designed poses.

        > *Input arguments*

        * `model_name` (*type:* `str`): ID name of the Gazebo model
        * `pose` (*type:* `list`): 6D pose vector
        """
        if not self.is_asset(model_name):
            self.add_gazebo_model_as_asset(model_name)
        model = SimulationModel.from_gazebo_model(model_name)

        # Model with the same name is already in the world list
        # Add ID identifier to avoid errors when spawning
        i = 0
        new_model_name = '{}'.format(model_name)
        while self._simulation_entity.model_exists(new_model_name):
            i += 1
            new_model_name = '{}_{}'.format(model_name, i)

        model.name = new_model_name
        # Add new model
        self.add_model(model, pose)
        PCG_ROOT_LOGGER.info(
            'Model <{}> added to world description'.format(model_name))

    def add_gazebo_model_as_asset(self, gazebo_model_name):
        """Create a model asset by importing a Gazebo model that already
        exists in the resources path of the catkin workspace. The model's
        SDF file will be parsed and converted into a
        `pcg_gazebo.simulation.SimulationModel` instance.

        Models that include lights can also be added, but will not be
        considered assets, they will just be included into the generated
        world SDF file.

        > *Input arguments*

        * `gazebo_model_name` (*type:* `str`): ID name from the Gazebo model
        to be imported

        > *Returns*

        `True` if Gazebo model could be included in the assets list.
        """
        from ..simulation import get_gazebo_model_sdf
        sdf = get_gazebo_model_sdf(gazebo_model_name)

        if hasattr(sdf, 'lights') and sdf.lights is not None:
            return self.add_lights_from_gazebo_model(gazebo_model_name)
        else:
            try:
                model = SimulationModel.from_gazebo_model(gazebo_model_name)
            except ValueError as ex:
                PCG_ROOT_LOGGER.error(
                    'Error loading Gazebo model <{}>, message={}'.format(
                        gazebo_model_name, str(ex)))
                return False

            if model is None:
                PCG_ROOT_LOGGER.error(
                    'Gazebo model with name <{}> '
                    'could not be found'.format(gazebo_model_name))
                return False
            self.add_asset(model)
        return True

    def is_asset(self, name):
        """Return `True` if the model identified by the string `name`
        is part of the list of assets.

        > *Input arguments*

        * `name` (*type:* `str`): Name of the model
        """
        return name in self._assets.tags

    def remove_asset(self, name):
        """Remove model asset from the list of assets.

        > *Input arguments*

        * `name` (*type:* `str`): Name of the model

        > *Returns*

        `True`, if model could be removed.
        """
        return self._assets.remove(name)

    def add_model(self, model, poses):
        """Add an instance of `pcg_gazebo.simulation.SimulationModel` to
        the world in designed poses.

        > *Input arguments*

        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`):
        Parameter description
        * `poses` (*type:* `list`): List of 6D pose vectors
        """
        self.add_asset(model)
        self.add_engine(
            tag=generate_random_string(5),
            engine_name='fixed_pose',
            models=[model.name],
            poses=poses)

    def add_lights_from_gazebo_model(self, model_name):
        """Add light models to the generated world from a Gazebo model.

        > *Input arguments*

        * `model_name` (*type:* `str`): Name of the Gazebo model

        > *Returns*

        `True` if the lights could be parsed and added to the world.
        """
        from ..simulation import get_gazebo_model_sdf
        sdf = get_gazebo_model_sdf(model_name)
        if sdf is None:
            PCG_ROOT_LOGGER.error(
                'Model {} is not a Gazebo model'.format(model_name))
            return False
        if sdf.lights is not None:
            PCG_ROOT_LOGGER.info(
                'Input Gazebo model {} contains'
                ' lights, adding to world'.format(
                    model_name))
            for light in sdf.lights:
                PCG_ROOT_LOGGER.info('Add light {}, type={}'.format(
                    light.name, light.type))
                self._simulation_entity.add_light(light.name, Light.from_sdf(
                    light))
            return True
        else:
            PCG_ROOT_LOGGER.error(
                'Input Gazebo model contains no '
                'lights, model_name={}'.format(model_name))
            return False

    def run_engines(self, attach_models=False):
        """Run all the model placement engines and add the generated
        models in the internal instance of the world representation.

        > *Input arguments*

        * `attach_models` (*type:* `bool`, *default:* `False`): Attach
        the generated models to the existent list of models in the world

        > *Returns*

        `True` if all engines ran successfully.
        """
        if self._simulation_entity is None:
            self.init()

        if self._engines.size == 0:
            PCG_ROOT_LOGGER.warning('No engines found')
            return False
        if not attach_models:
            self._simulation_entity.reset_models()
            PCG_ROOT_LOGGER.info('List of models is now empty')

        models = list()
        # Run the fixed pose engines first
        PCG_ROOT_LOGGER.info('Run fixed-pose engines')
        for tag in self._engines.tags:
            engine = self._engines.get(tag)
            if engine.label == 'fixed_pose':
                models = engine.run()
                if models is not None:
                    for model in models:
                        if not self._add_asset_to_simulation_entity(model):
                            PCG_ROOT_LOGGER.error(
                                'Cannot add asset <{}>'.format(
                                    model.name))
        # Run all other engines
        PCG_ROOT_LOGGER.info('Run other engines')
        for tag in self._engines.tags:
            engine = self._engines.get(tag)
            if engine.label != 'fixed_pose':
                PCG_ROOT_LOGGER.info(
                    'Running engine, type={}'.format(
                        engine.label))
                engine.set_fixed_pose_models(
                    list(self._simulation_entity.models.values()))
                models = engine.run()
                if models is not None:
                    for model in models:
                        if not self._add_asset_to_simulation_entity(model):
                            PCG_ROOT_LOGGER.error(
                                'Cannot add asset <{}>'.format(
                                    model.name))

        PCG_ROOT_LOGGER.info(
            'Model placement finished, # models={}, model_names={}'.format(
                len(self._simulation_entity.models),
                list(self._simulation_entity.models.keys())))
        return True

    def init(self):
        raise NotImplementedError(
            'The derived class should implement this function')

    def from_yaml(self, filename):
        config = load_yaml(filename)
        self.from_dict(config)

    def from_dict(self, config):
        assert isinstance(config, dict), \
            'Input configuration must be provided as a dictionary'

        if 'name' in config:
            self.name = config['name']
            PCG_ROOT_LOGGER.info('Generator name: {}'.format(self._name))

        if 'ground_plane' in config:
            for tag in config['ground_plane']:
                self.set_model_as_ground_plane(tag)

                PCG_ROOT_LOGGER.info(
                    'Set model {} as ground plane'.format(tag))

        if 'assets' in config:
            self.assets.from_dict(config['assets'])

        if 'engines' in config:
            if not isinstance(config['engines'], list):
                PCG_ROOT_LOGGER.error(
                    '<engines> element in dictionary must be a list')
            else:
                self.engines.from_dict(config['engines'])

        if 'constraints' in config:
            if not isinstance(config['constraints'], list):
                PCG_ROOT_LOGGER.error(
                    '<constraints> element in dictionary must be a list')
            for elem in config['constraints']:
                assert isinstance(
                    elem, dict), 'Constraint description ' \
                    'is not a dictionary={}'.format(elem)
                self.engines.add_constraint(**elem)

        if 'lights' in config:
            PCG_ROOT_LOGGER.info('Lights:')
            for light in config['lights']:
                if 'name' not in light:
                    PCG_ROOT_LOGGER.error('Light item has no name')
                    continue
                if is_gazebo_model(light['name']):
                    self.add_lights_from_gazebo_model(light['name'])
