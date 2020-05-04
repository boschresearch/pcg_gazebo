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
from .model import Model
from .light import Light


class Insertions(XMLBase):
    _NAME = 'insertions'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        model=dict(creator=Model, n_elems='+', optional=True),
        light=dict(creator=Light, n_elems='+', optional=True)
    )

    def __init__(self):
        super(Insertions, self).__init__()
        self.reset()

    @property
    def models(self):
        return self._get_child_element('model')

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
