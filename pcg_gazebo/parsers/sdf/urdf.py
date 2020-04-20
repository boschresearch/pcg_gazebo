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
from ..urdf import Transmission, Mimic, Link, SafetyController


class URDF(XMLBase):
    _NAME = 'urdf'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        transmission=dict(
            creator=Transmission,
            n_elems='+',
            optional=True,
            mode='model'),
        link=dict(
            creator=Link,
            n_elems='+',
            optional=True,
            mode='model'),
        mimic=dict(
            creator=Mimic,
            optional=True,
            mode='joint'),
        safety_controller=dict(
            creator=SafetyController,
            optional=True,
            mode='joint'))

    _MODES = ['model', 'joint']

    def __init__(self, mode='model'):
        super(URDF, self).__init__()
        self.reset(mode=mode)

    @property
    def mimic(self):
        return self._get_child_element('mimic')

    @mimic.setter
    def mimic(self, value):
        if self._mode != 'joint':
            self.reset(mode='joint')
        self._add_child_element('mimic', value)

    @property
    def safety_controller(self):
        return self._get_child_element('safety_controller')

    @safety_controller.setter
    def safety_controller(self, value):
        if self._mode != 'joint':
            self.reset(mode='joint')
        self._add_child_element('safety_controller', value)

    @property
    def transmissions(self):
        return self._get_child_element('transmission')

    @property
    def links(self):
        return self._get_child_element('link')

    def add_transmission(self, name, transmission=None):
        if self._mode != 'model':
            self.reset(mode='model')
        if self.transmissions is not None:
            for elem in self.transmissions:
                if elem.name == name:
                    print(
                        'Transmission element with name {}'
                        ' already exists'.format(name))
                    return
        if transmission is not None:
            self._add_child_element('transmission', transmission)
        else:
            transmission = Transmission()
            self._add_child_element('transmission', transmission)
        self.children['transmission'][-1].name = name

    def get_transmission_by_name(self, name):
        if self.transmissions is None:
            return None
        else:
            for elem in self.transmissions:
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

    def _add_child_element(self, name, value=None):
        assert name in self._CHILDREN_CREATORS, \
            'Input URDF tag {} cannot be added'.format(name)

        elem = self._CHILDREN_CREATORS[name]['creator']()
        assert elem is not None, 'Invalid URDF element'

        if value is not None:
            if elem.has_value():
                if isinstance(value, dict):
                    if 'attributes' in value:
                        for tag in value['attributes']:
                            setattr(elem, tag, value['attributes'][tag])
                    if 'value' in value:
                        elem.value = value['value']
                    else:
                        for tag in value:
                            if tag == 'attributes':
                                continue
                            elem._add_child_element(tag, value[tag])
                else:
                    elem.value = value
            else:
                if issubclass(value.__class__, XMLBase):
                    if name == value._NAME:
                        elem = value
                elif isinstance(value, dict):
                    for tag in value:
                        if tag == 'attributes':
                            for att in value[tag]:
                                setattr(elem, att, value[tag][att])
                        else:
                            if elem._NAME != 'empty':
                                if isinstance(value[tag], list):
                                    if sum([isinstance(x.__class__, dict)
                                            for x in value[tag]]) == 0:
                                        elem._add_child_element(
                                            tag, value[tag])
                                    else:
                                        for item in value[tag]:
                                            elem._add_child_element(tag, item)
                                else:
                                    elem._add_child_element(tag, value[tag])

            if 'n_elems' in self._CHILDREN_CREATORS[name]:
                if self._CHILDREN_CREATORS[name]['n_elems'] == '+':
                    if name not in self.children:
                        self.children[name] = list()
                    self.children[name].append(elem)
                else:
                    self.children[name] = elem
            else:
                self.children[name] = elem

            return True
