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
from .pose import Pose
from .geometry import Geometry
from .max_contacts import MaxContacts
from .surface import Surface
from .laser_retro import LaserRetro
from ...utils import generate_random_string


class Collision(XMLBase):
    _NAME = 'collision'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        geometry=dict(
            creator=Geometry,
            mode='link'),
        pose=dict(
            creator=Pose,
            n_elems=1,
            mode='link',
            optional=True),
        max_contacts=dict(
            creator=MaxContacts,
            n_elems=1,
            default=[10],
            mode='link',
            optional=True),
        surface=dict(
            creator=Surface,
            mode='link',
            optional=True),
        laser_retro=dict(
            creator=LaserRetro,
            default=[0],
            mode='link',
            optional=True)
    )

    _ATTRIBUTES = dict(
        name='collision'
    )

    _MODES = ['string', 'link', 'state']

    def __init__(self, mode='link', default='__default__'):
        super(Collision, self).__init__()
        if mode == 'string':
            self._default = default
            self._value = default
            self._VALUE_TYPE = 'string'
        else:
            self._default = '__default__'
            self._value = None
        self.reset(mode=mode)

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert isinstance(value, str), 'Collision name must be a string'
        assert len(value) > 0, 'Collision name string is empty'
        self.attributes['name'] = value

    @property
    def pose(self):
        return self._get_child_element('pose')

    @pose.setter
    def pose(self, vec):
        self._add_child_element('pose', vec)

    @property
    def geometry(self):
        return self._get_child_element('geometry')

    @geometry.setter
    def geometry(self, value):
        self._add_child_element('geometry', value)

    @property
    def max_contacts(self):
        return self._get_child_element('max_contacts')

    @max_contacts.setter
    def max_contacts(self, value):
        self._add_child_element('max_contacts', value)

    @property
    def surface(self):
        return self._get_child_element('surface')

    @surface.setter
    def surface(self, value):
        self._add_child_element('surface', value)

    @property
    def laser_retro(self):
        return self._get_child_element('laser_retro')

    @laser_retro.setter
    def laser_retro(self, value):
        self._add_child_element('laser_retro', value)

    def reset(self, mode=None, with_optional_elements=False):
        if mode is not None:
            if mode not in self._MODES:
                self.log_error(
                    'Mode can either be boolean or vector',
                    raise_exception=True,
                    exception_type=AssertionError)
            self._mode = mode
        if self._mode == 'string':
            self.children = dict()
            self._value = self._default
            self._VALUE_TYPE = 'string'
        else:
            self._VALUE_TYPE = ''
            self._value = None
            XMLBase.reset(
                self, mode=mode,
                with_optional_elements=with_optional_elements)

    def _set_value(self, value):
        if self._mode != 'string':
            self.reset(mode='string')
        assert self._is_string(value), \
            '[{}] Input value must be string for {},' \
            ' received={}, type={}'.format(
                self.xml_element_name, self._NAME, value, type(value))
        self._value = str(value)

    def is_valid(self):
        if self._mode == 'string':
            if not self._is_string(self._value):
                self.log_error(
                    'Invalid string object, value={}'.format(
                        self._value))
                return False
            else:
                return True
        else:
            return XMLBase.is_valid(self)

    def get_formatted_value_as_str(self):
        if self._mode == 'string':
            assert self.is_valid(), 'Invalid string value'
            return '{}'.format(self._value)
        return None

    def random(self):
        if self._mode == 'string':
            self._set_value(generate_random_string(5))
        else:
            XMLBase.random(self)
