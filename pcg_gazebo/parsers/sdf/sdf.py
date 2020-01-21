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

from ..types import XMLBase
from .world import World
from .model import Model
from .light import Light


class SDF(XMLBase):
    _NAME = 'sdf'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        version='1.6'
    )

    _CHILDREN_CREATORS = dict(
        world=dict(creator=World, optional=True),
        model=dict(creator=Model, n_elems='+', optional=True),
        light=dict(creator=Light, n_elems='+', optional=True)
    )

    _MODES = ['world', 'model', 'light', 'actor']

    _VERSIONS = ['1.4', '1.5', '1.6']

    def __init__(self, mode='world'):
        XMLBase.__init__(self)
        self.reset(mode)

    @property
    def version(self):
        return self.attributes['version']

    @version.setter
    def version(self, value):
        assert str(value) in self._VERSIONS, '{} not a' \
            ' valid XML version, versions={}'.format(
                value, self._VERSIONS)
        self.attributes['version'] = str(value)

    @property
    def world(self):
        return self._get_child_element('world')

    @world.setter
    def world(self, value):
        self._add_child_element('world', value)

    @property
    def models(self):
        return self._get_child_element('model')

    @property
    def lights(self):
        return self._get_child_element('light')

    def add_model(self, name=None, model=None):
        if model is None:
            model = Model()
        self._add_child_element('model', model)
        if name is not None:
            self.children['model'][-1].name = name

    def add_light(self, name=None, light=None):
        if light is None:
            light = Light()
        self._add_child_element('light', light)
        if name is not None:
            self.children['light'][-1].name = name
