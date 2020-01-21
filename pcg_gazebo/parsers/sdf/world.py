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
from .physics import Physics
from .model import Model
from .include import Include
from .gravity import Gravity
from .plugin import Plugin
from .light import Light
from .scene import Scene
from .gui import GUI


class World(XMLBase):
    _NAME = 'world'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        physics=dict(creator=Physics),
        gravity=dict(creator=Gravity, default=[[0, 0, -9.8]]),
        model=dict(creator=Model, n_elems='+', optional=True),
        include=dict(creator=Include, n_elems='+', optional=True),
        plugin=dict(creator=Plugin, n_elems='+', optional=True),
        light=dict(creator=Light, n_elems='+', optional=True),
        scene=dict(creator=Scene, optional=True),
        gui=dict(creator=GUI, optional=True)
    )

    _ATTRIBUTES = dict(
        name='default'
    )

    def __init__(self):
        XMLBase.__init__(self)
        self.reset()

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert isinstance(value, str), 'Name should be a string'
        assert len(value) > 0, 'Name cannot be an empty string'
        self.attributes['name'] = value

    @property
    def gui(self):
        return self._get_child_element('gui')

    @gui.setter
    def gui(self, value):
        self._add_child_element('gui', value)

    @property
    def scene(self):
        return self._get_child_element('scene')

    @scene.setter
    def scene(self, value):
        self._add_child_element('scene', value)

    @property
    def physics(self):
        return self._get_child_element('physics')

    @physics.setter
    def physics(self, value):
        self._add_child_element('physics', value)

    @property
    def gravity(self):
        return self._get_child_element('gravity')

    @gravity.setter
    def gravity(self, value):
        self._add_child_element('gravity', value)

    @property
    def models(self):
        return self._get_child_element('model')

    @property
    def includes(self):
        return self._get_child_element('include')

    @property
    def plugins(self):
        return self._get_child_element('plugin')

    @property
    def lights(self):
        return self._get_child_element('light')

    def add_model(self, name, model=None):
        if self.models is not None:
            for elem in self.models:
                if elem.name == name:
                    print(
                        'Model element with name {}'
                        ' already exists'.format(name))
                    return
        if model is not None:
            self._add_child_element('model', model)
        else:
            model = Model()
            self._add_child_element('model', model)
        self.children['model'][-1].name = name

    def get_model_by_name(self, name):
        if self.models is None:
            return None
        else:
            for elem in self.models:
                if elem.name == name:
                    return elem
        return None

    def add_include(self, name=None, include=None):
        if include is not None:
            self._add_child_element('include', include)
        else:
            include = Include()
            self._add_child_element('include', include)

    def add_plugin(self, name, plugin=None):
        if plugin is not None:
            self._add_child_element('plugin', plugin)
        else:
            plugin = Plugin()
            self._add_child_element('plugin', plugin)

    def add_light(self, name, light=None):
        if self.lights is not None:
            for elem in self.lights:
                if elem.name == name:
                    print(
                        'Light element with name {}'
                        ' already exists'.format(name))
                    return
        if light is not None:
            self._add_child_element('light', light)
        else:
            light = Light()
            self._add_child_element('light', light)
        self.children['light'][-1].name = name

    def get_light_by_name(self, name):
        if self.lights is None:
            return None
        else:
            for elem in self.lights:
                if elem.name == name:
                    return elem
        return None
