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
from ..utils import load_yaml
from .engines import create_engine
from ._collection_manager import _CollectionManager
from .constraints_manager import ConstraintsManager
from .assets_manager import AssetsManager
from .collision_checker import CollisionChecker
from ..log import PCG_ROOT_LOGGER
from ..utils import generate_random_string


class EngineManager(_CollectionManager):
    def __init__(self):
        super(EngineManager, self).__init__()

        self._constraints_manager = ConstraintsManager()
        self._assets_manager = AssetsManager.get_instance()
        self._collision_checker = CollisionChecker()

    @property
    def constraints_manager(self):
        return self._constraints_manager

    def add(self, tag, engine_name, models, **kwargs):
        """Add a new model creator engine to the internal engines list.

        > *Input arguments*

        * `engine_name` (*type:* `str`): Name of the engine class
        to be created
        * `models` (*type:* list of `str`): Name of the models
        that will be assets to the created engine
        * `kwargs` (*type:* `dict`): Input arguments to the
        created engine.
        """
        if self.has_element(tag):
            PCG_ROOT_LOGGER.error(
                'Engine with tag <{}> already exists'.format(tag))
            return False
        input_args = kwargs
        input_args['models'] = models
        input_args['assets_manager'] = self._assets_manager
        input_args['callback_fcn_get_constraint'] = \
            self._constraints_manager.get
        input_args['collision_checker'] = self._collision_checker
        self._collection[tag] = create_engine(engine_name, **input_args)
        PCG_ROOT_LOGGER.info(
            'New model creator engine added, type={}, tag={}'.format(
                engine_name, tag))
        return True

    def from_dict(self, config):
        assert isinstance(config, dict) or isinstance(config, list), \
            'Input must be either a dictionary or a list of dictionaries'

        if isinstance(config, list):
            for elem in config:
                self.from_dict(elem)
        else:
            if 'tag' not in config:
                config['tag'] = generate_random_string(10)
            if not self.add(**config):
                PCG_ROOT_LOGGER.error('Failed to add engine={}'.format(config))

    def from_yaml(self, filename):
        """Load the engines from a YAML file.

        > *Input arguments*

        * `filename` (*type:* `str`): YAML filename.
        """
        config = load_yaml(filename)
        self.from_dict(config)

    def add_constraint(self, name, type, **kwargs):
        return self._constraints_manager.add(name=name, type=type, **kwargs)

    def get_constraint(self, name):
        return self._constraints_manager.get(name)

    def has_constraint(self, name):
        return self._constraints_manager.has_element(name)
