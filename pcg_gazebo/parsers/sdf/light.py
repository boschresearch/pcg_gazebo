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
from ...utils import is_string
from .pose import Pose
from .cast_shadows import CastShadows
from .diffuse import Diffuse
from .specular import Specular
from .attenuation import Attenuation
from .direction import Direction
from .spot import Spot
from .frame import Frame


class Light(XMLBase):
    _NAME = 'light'
    _TYPE = 'sdf'

    _ATTRIBUTES = dict(
        name='default',
        type='point'
    )

    _ATTRIBUTES_MODES = dict(
        name=['light', 'state'],
        type=['light']
    )

    _CHILDREN_CREATORS = dict(
        cast_shadows=dict(creator=CastShadows, default=[False], mode='light'),
        diffuse=dict(creator=Diffuse, default=[[1, 1, 1, 1]], mode='light'),
        specular=dict(
            creator=Specular, default=[[0.1, 0.1, 0.1, 1]], mode='light'),
        attenuation=dict(creator=Attenuation, mode='light'),
        direction=dict(creator=Direction, default=[[0, 0, -1]], mode='light'),
        spot=dict(creator=Spot, optional=True, mode='light'),
        pose=dict(creator=Pose, optional=True),
        frame=dict(creator=Frame, optional=True, n_elems='+', mode='state')
    )

    _MODES = ['light', 'state']

    def __init__(self, mode='light'):
        super(Light, self).__init__()
        self.reset(mode=mode)

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert is_string(value), 'Name should'\
            ' be a string, received={}, type={}'.format(
                value, type(value))
        assert len(value) > 0, \
            'Name string should not be empty'
        self.attributes['name'] = value

    @property
    def type(self):
        return self.attributes['type']

    @type.setter
    def type(self, value):
        if self._mode != 'light':
            self._mode = 'light'
        assert is_string(value), 'Name should' \
            ' be a string, received={}, type={}'.format(
                value, type(value))
        assert len(value) > 0, \
            'Name string should not be empty'
        assert value in ['point', 'directional', 'spot'], \
            'Valid light options are point, directional and spot'
        self.attributes['type'] = value

    @property
    def pose(self):
        return self._get_child_element('pose')

    @pose.setter
    def pose(self, value):
        self._add_child_element('pose', value)

    @property
    def cast_shadows(self):
        return self._get_child_element('cast_shadows')

    @cast_shadows.setter
    def cast_shadows(self, value):
        if self._mode != 'light':
            self._mode = 'light'
        self._add_child_element('cast_shadows', value)

    @property
    def diffuse(self):
        return self._get_child_element('diffuse')

    @diffuse.setter
    def diffuse(self, value):
        if self._mode != 'light':
            self._mode = 'light'
        self._add_child_element('diffuse', value)

    @property
    def specular(self):
        return self._get_child_element('specular')

    @specular.setter
    def specular(self, value):
        if self._mode != 'light':
            self._mode = 'light'
        self._add_child_element('specular', value)

    @property
    def attenuation(self):
        return self._get_child_element('attenuation')

    @attenuation.setter
    def attenuation(self, value):
        if self._mode != 'light':
            self._mode = 'light'
        self._add_child_element('attenuation', value)

    @property
    def direction(self):
        return self._get_child_element('direction')

    @direction.setter
    def direction(self, value):
        if self._mode != 'light':
            self._mode = 'light'
        self._add_child_element('direction', value)

    @property
    def spot(self):
        return self._get_child_element('spot')

    @spot.setter
    def spot(self, value):
        if self._mode != 'light':
            self._mode = 'light'
        self._add_child_element('spot', value)

    @property
    def frames(self):
        return self._get_child_element('frame')

    def add_frame(self, name=None, frame=None):
        if self._mode != 'state':
            self._mode = 'state'
        if self.frames is not None:
            for elem in self.frames:
                if elem.name == name:
                    print(
                        'Frame element with name {}'
                        ' already exists'.format(name))
                    return
        if frame is not None:
            self._add_child_element('frame', frame)
        else:
            frame = Frame()
            self._add_child_element('frame', frame)
        self._children['frame'][-1].name = name
