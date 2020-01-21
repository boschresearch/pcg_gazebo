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
from ..utils import is_string
from .assets_manager import AssetsManager
from .engine_manager import EngineManager
from ..simulation import ModelGroup
from ..log import PCG_ROOT_LOGGER


class ModelGroupGenerator(object):
    def __init__(self, name='generator'):
        self._name = ''
        self._engines_manager = EngineManager()
        self._assets_manager = AssetsManager.get_instance()
        self.name = name

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        assert is_string(value), \
            'Model name should be a string'
        assert len(value) > 0, 'Model name cannot be an empty string'
        self._name = value

    @property
    def assets(self):
        return self._assets_manager

    @property
    def engines(self):
        return self._engines_manager

    @property
    def constraints(self):
        return self._engines_manager.constraints_manager

    def add_engine(self, tag, engine_name, models, **kwargs):
        return self._engines_manager.add(tag, engine_name, models, **kwargs)

    def get_engine(self, tag):
        return self._engines_manager.get(tag)

    def get_asset(self, name):
        return self._assets_manager.get(name)

    def set_asset_as_ground_plane(self, name):
        return self._assets_manager.set_asset_as_ground_plane(name)

    def add_gazebo_model_as_asset(self, gazebo_model_name):
        return self._assets_manager.add(gazebo_model_name)

    def is_asset(self, name):
        return self._assets_manager.has_element(name)

    def add_asset(
            self,
            description,
            tag=None,
            parameters=None,
            include_dir=None):
        return self._assets_manager.add(
            description,
            tag=tag,
            parameters=parameters,
            include_dir=include_dir)

    def add_constraint(self, name, type, **kwargs):
        return self._engines_manager.add_constraint(
            name=name, type=type, **kwargs)

    def get_constraint(self, name):
        return self._engines_manager.get_constraint(name)

    def run(self, group_name='default', pose=[0, 0, 0, 0, 0, 0]):
        if self._assets_manager.has_element(
            group_name) and \
                self._assets_manager.is_model_group(group_name):
            model_group = self._assets_manager.get(group_name)
        else:
            model_group = ModelGroup(name=group_name)

        if self.engines.size == 0:
            PCG_ROOT_LOGGER.warning(
                '[{}] No engines found for model group generator'.format(
                    self.name))
            return None

        fixed_pose_models = list()
        models = list()
        # TODO Pass pose offset for the group to the engine
        # Run the fixed pose engines first
        PCG_ROOT_LOGGER.info('[{}] Run fixed-pose engines'.format(self.name))
        for tag in self._engines_manager.tags:
            engine = self._engines_manager.get(tag)
            if engine.label == 'fixed_pose':
                models = engine.run()
                if models is not None:
                    for model in models:
                        PCG_ROOT_LOGGER.info(
                            '[{}] Adding model <{}> to model '
                            'group <{}>'.format(
                                self.name, model.name, group_name))
                        model_group.add_model(model.name, model)
                        fixed_pose_models.append(model)

        # Run all other engines
        PCG_ROOT_LOGGER.info('[{}] Run other engines'.format(self.name))
        for tag in self._engines_manager.tags:
            engine = self._engines_manager.get(tag)
            if engine.label != 'fixed_pose':
                PCG_ROOT_LOGGER.info('[{}] Running engine, type={}'.format(
                    self.name, engine.label))
                engine.set_fixed_pose_models(fixed_pose_models)
                models = engine.run()
                if models is not None:
                    for model in models:
                        PCG_ROOT_LOGGER.info(
                            '[{}] Adding model <{}> to model '
                            'group <{}>'.format(
                                self.name, model.name, group_name))
                        model_group.add_model(model.name, model)

        PCG_ROOT_LOGGER.info(
            '[{}] Model generation finished, group={}, '
            '# models={}, model_names={}'.format(
                self.name, model_group.name, len(
                    model_group.models), list(
                    model_group.models.keys())))

        model_group.pose = pose
        return model_group

    @staticmethod
    def from_dict(config):
        assert isinstance(config, dict), \
            'Input configuration must be provided as a dictionary'

        name = 'generator'
        if 'name' in config:
            name = config['name']
        generator = ModelGroupGenerator(name=name)
        if 'assets' in config:
            generator._assets_manager.from_dict(config['assets'])

        if 'engines' in config:
            if not isinstance(config['engines'], list):
                PCG_ROOT_LOGGER.error(
                    '<engines> element in dictionary must be a list')
            else:
                generator._engines_manager.from_dict(config['engines'])

        if 'constraints' in config:
            if not isinstance(config['constraints'], list):
                PCG_ROOT_LOGGER.error(
                    '<constraints> element in dictionary must be a list')
            for elem in config['constraints']:
                assert isinstance(
                    elem, dict), 'Constraint description ' \
                    'is not a dictionary={}'.format(elem)
                generator._engines_manager.add_constraint(**elem)
        return generator
