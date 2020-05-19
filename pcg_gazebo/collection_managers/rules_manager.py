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
from ._collection_manager import _CollectionManager
from ..generators.rules import create_rule, Rule
from ..utils import load_yaml, generate_random_string
from ..log import PCG_ROOT_LOGGER


class RulesManager(_CollectionManager):
    def __init__(self):
        super(RulesManager, self).__init__()

    @staticmethod
    def get_instance():
        if RulesManager._INSTANCE is None:
            RulesManager._INSTANCE = RulesManager()
        return RulesManager._INSTANCE

    def add(self, name=None, type=None, rule_obj=None, **kwargs):
        """Add a new positioning rule class to the internal
        rules list.

        > *Input arguments*

        * `name` (*type:* `str`): ID name for the rule class instance
        * `type` (*type:* `str`): Name of the rules class to be created
        * `kwargs` (*type:* `dict`): Input arguments for the rule class
        to be created
        """
        new_role = create_rule(type, **kwargs)
        if name is None:
            name = generate_random_string(5)
            while self.has_element(name):
                name = generate_random_string(5)
            PCG_ROOT_LOGGER.info(
                'Since rule tag was not provided, '
                'an unique random tag has been '
                'generated, name={}'.format(name))
        if self.has_element(name):
            if self._collection[name] != new_role:
                PCG_ROOT_LOGGER.error(
                    'Rule with name <{}> already'
                    ' exists and has different parameters'.format(
                        name))
                return False
            else:
                return True
        if rule_obj is not None and isinstance(rule_obj, Rule):
            self._collection[name] = rule_obj
        else:
            self._collection[name] = create_rule(type, **kwargs)
        return True

    def from_dict(self, config):
        assert isinstance(config, dict) or isinstance(config, list), \
            'Input must be either a dictionary or a list of dictionaries'

        if isinstance(config, list):
            for elem in config:
                self.from_dict(elem)
        else:
            if not self.add(**config):
                PCG_ROOT_LOGGER.error(
                    'Failed to parse rule'
                    ' configuration={}'.format(config))

    def from_yaml(self, filename):
        config = load_yaml(filename)
        self.from_dict(config)
