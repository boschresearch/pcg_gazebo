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
from ._generator import _Generator
from ..simulation import ModelGroup


class ModelGroupGenerator(_Generator):
    def __init__(self, name='generator', **kwargs):
        super(ModelGroupGenerator, self).__init__(name=name, **kwargs)

    def init(self, name=None):
        if name is None:
            name = self._name
        if self.assets.has_element(name) and \
                self.assets.is_model_group(name):
            self._simulation_entity = self.assets.get(name)
        else:
            self._simulation_entity = ModelGroup(name=name)

        self.engines.reset_engines()

    def run(self, group_name='default', pose=[0, 0, 0, 0, 0, 0]):
        self.init(group_name)
        self.run_engines()
        self._simulation_entity.pose = pose
        return self._simulation_entity
