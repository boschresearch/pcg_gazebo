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
from .actor import Actor
from .include import Include
from .gravity import Gravity
from .plugin import Plugin
from .light import Light
from .scene import Scene
from .gui import GUI
from .state import State
from .spherical_coordinates import SphericalCoordinates
from .wind import Wind
from .magnetic_field import MagneticField
from .atmosphere import Atmosphere


class World(XMLBase):
    _NAME = 'world'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        atmosphere=dict(creator=Atmosphere, optional=False),
        wind=dict(creator=Wind, optional=False),
        physics=dict(creator=Physics),
        gravity=dict(creator=Gravity, default=[[0, 0, -9.8]]),
        model=dict(creator=Model, n_elems='+', optional=True),
        actor=dict(creator=Actor, n_elems='+', optional=True),
        include=dict(creator=Include, n_elems='+', optional=True),
        plugin=dict(creator=Plugin, n_elems='+', optional=True),
        light=dict(creator=Light, n_elems='+', optional=True),
        scene=dict(creator=Scene, optional=True),
        gui=dict(creator=GUI, optional=True),
        state=dict(creator=State, optional=True, n_elems='+'),
        spherical_coordinates=dict(
            creator=SphericalCoordinates, optional=True),
        magnetic_field=dict(
            creator=MagneticField,
            default=[[6e-6, 2.3e-5, -4.2e-5]],
            optional=True)
    )

    _ATTRIBUTES = dict(
        name='default'
    )

    def __init__(self):
        super(World, self).__init__()
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
    def atmosphere(self):
        return self._get_child_element('atmosphere')

    @atmosphere.setter
    def atmosphere(self, value):
        self._add_child_element('atmosphere', value)

    @property
    def wind(self):
        return self._get_child_element('wind')

    @wind.setter
    def wind(self, value):
        self._add_child_element('wind', value)

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
    def spherical_coordinates(self):
        return self._get_child_element('spherical_coordinates')

    @spherical_coordinates.setter
    def spherical_coordinates(self, value):
        self._add_child_element('spherical_coordinates', value)

    @property
    def magnetic_field(self):
        return self._get_child_element('magnetic_field')

    @magnetic_field.setter
    def magnetic_field(self, value):
        self._add_child_element('magnetic_field', value)

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

    @property
    def states(self):
        return self._get_child_element('state')

    @property
    def actors(self):
        return self._get_child_element('actor')

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

    def add_state(self, name, state=None):
        if self.states is not None:
            for elem in self.states:
                if elem.world_name == name:
                    print(
                        'State element with name {}'
                        ' already exists'.format(name))
                    return
        if state is not None:
            self._add_child_element('state', state)
        else:
            state = State()
            self._add_child_element('state', state)
        self.children['state'][-1].world_name = name

    def get_state_by_name(self, name):
        if self.states is None:
            return None
        else:
            for elem in self.states:
                if elem.world_name == name:
                    return elem
        return None

    def add_actor(self, name, actor=None):
        if self.actors is not None:
            for elem in self.actors:
                if elem.name == name:
                    print(
                        'State element with name {}'
                        ' already exists'.format(name))
                    return
        if actor is not None:
            self._add_child_element('actor', actor)
        else:
            actor = Actor()
            self._add_child_element('actor', actor)
        self.children['actor'][-1].world_name = name

    def get_actor_by_name(self, name):
        if self.actors is None:
            return None
        else:
            for elem in self.actors:
                if elem.name == name:
                    return elem
        return None
