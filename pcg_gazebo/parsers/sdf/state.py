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
from ..types import XMLBase
from .sim_time import SimTime
from .wall_time import WallTime
from .real_time import RealTime
from .iterations import Iterations
from .insertions import Insertions
from .deletions import Deletions
from .model import Model
from .light import Light
from .link import Link


class State(XMLBase):
    _NAME = 'state'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        world_name='__default__'
    )

    _CHILDREN_CREATORS = dict(
        sim_time=dict(creator=SimTime, optional=True, default=[[0, 0]]),
        wall_time=dict(creator=WallTime, optional=True, default=[[0, 0]]),
        real_time=dict(creator=RealTime, optional=True, default=[[0, 0]]),
        iterations=dict(creator=Iterations, default=[0]),
        insertions=dict(creator=Insertions, optional=True),
        deletions=dict(creator=Deletions, optional=True),
        model=dict(
            creator=Model, n_elems='+', optional=True, default=['state']),
        link=dict(
            creator=Model, n_elems='+', optional=True, default=['state']),
        light=dict(
            creator=Light, n_elems='+', optional=True, default=['state'])
    )

    def __init__(self):
        super(State, self).__init__()
        self.reset()

    @property
    def world_name(self):
        return self.attributes['world_name']

    @world_name.setter
    def world_name(self, value):
        assert self._is_string(value), \
            'World name must be a string'
        self.attributes['world_name'] = value

    @property
    def sim_time(self):
        return self._get_child_element('sim_time')

    @sim_time.setter
    def sim_time(self, value):
        self._add_child_element('sim_time', value)

    @property
    def wall_time(self):
        return self._get_child_element('wall_time')

    @wall_time.setter
    def wall_time(self, value):
        self._add_child_element('wall_time', value)

    @property
    def real_time(self):
        return self._get_child_element('real_time')

    @real_time.setter
    def real_time(self, value):
        self._add_child_element('real_time', value)

    @property
    def iterations(self):
        return self._get_child_element('iterations')

    @iterations.setter
    def iterations(self, value):
        self._add_child_element('iterations', value)

    @property
    def insertions(self):
        return self._get_child_element('insertions')

    @insertions.setter
    def insertions(self, value):
        self._add_child_element('insertions', value)

    @property
    def deletions(self):
        return self._get_child_element('deletions')

    @deletions.setter
    def deletions(self, value):
        self._add_child_element('deletions', value)

    @property
    def models(self):
        return self._get_child_element('model')

    @property
    def links(self):
        return self._get_child_element('link')

    @property
    def lights(self):
        return self._get_child_element('lights')

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

    def add_link(self, name, link=None):
        if self.links is not None:
            for elem in self.links:
                if elem.name == name:
                    print(
                        'Link element with name {}'
                        ' already exists'.format(name))
                    return
        if link is not None:
            self._add_child_element('link', link)
        else:
            link = Link()
            self._add_child_element('link', link)
        self.children['link'][-1].name = name

    def get_link_by_name(self, name):
        if self.links is None:
            return None
        else:
            for elem in self.links:
                if elem.name == name:
                    return elem
        return None
