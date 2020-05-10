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
from .actor import Actor


class SDF(XMLBase):
    _NAME = 'sdf'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        version='1.6'
    )

    _CHILDREN_CREATORS = dict(
        world=dict(creator=World, optional=True, mode='world'),
        model=dict(
            creator=Model, n_elems='+', optional=True, mode='model'),
        light=dict(
            creator=Light, n_elems='+', optional=True, mode='model'),
        actor=dict(
            creator=Actor, n_elems='+', optional=True, mode='model')
    )

    _MODES = ['world', 'model']

    _VERSIONS = ['1.4', '1.5', '1.6']

    def __init__(self, mode='world'):
        super(SDF, self).__init__()
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
        if self._mode != 'world':
            self._mode = 'world'
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

    @property
    def actors(self):
        return self._get_child_element('actor')

    def add_model(self, name=None, model=None):
        if self._mode != 'model':
            self._mode = 'model'
        if model is None:
            model = Model()
        self._add_child_element('model', model)
        if name is not None:
            self.children['model'][-1].name = name

    def add_light(self, name=None, light=None):
        if self._mode != 'model':
            self._mode = 'model'
        if light is None:
            light = Light()
        self._add_child_element('light', light)
        if name is not None:
            self.children['light'][-1].name = name

    def add_actor(self, name=None, actor=None):
        if self._mode != 'model':
            self._mode = 'model'
        if actor is None:
            actor = Actor()
        self._add_child_element('actor', actor)
        if name is not None:
            self.children['actor'][-1].name = name
