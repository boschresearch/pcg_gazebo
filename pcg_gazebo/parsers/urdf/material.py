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
from .color import Color
from .texture import Texture
from .gazebo import Gazebo


class Material(XMLBase):
    _NAME = 'material'
    _TYPE = 'urdf'

    _ATTRIBUTES = dict(
        name=''
    )

    _CHILDREN_CREATORS = dict(
        color=dict(creator=Color, mode='color'),
        texture=dict(creator=Texture, mode='texture'),
        gazebo=dict(
            creator=Gazebo,
            optional=True,
            default=['none', dict(
                lighting=None,
                emissive=None,
                script=None
            )])
    )

    _MODES = ['color', 'texture']

    def __init__(self, mode='color'):
        XMLBase.__init__(self)
        self.reset(mode)

    @property
    def name(self):
        return self.attributes['name']

    @name.setter
    def name(self, value):
        assert isinstance(value, str), 'Name input must be a string'
        self.attributes['name'] = value

    @property
    def color(self):
        assert self._mode == 'color', 'Material should be in color mode'
        return self._get_child_element('color')

    @color.setter
    def color(self, value):
        assert self._mode == 'color', 'Material should be in color mode'
        self._add_child_element('color', value)

    @property
    def texture(self):
        assert self._mode == 'texture', 'Material should be in texture mode'
        return self._get_child_element('texture')

    @texture.setter
    def texture(self, value):
        assert self._mode == 'texture', 'Material should be in texture mode'
        self._add_child_element('texture', value)

    @property
    def lighting(self):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo('none', dict(lighting=None))
        return self.children['gazebo']._get_child_element('lighting')

    @lighting.setter
    def lighting(self, value):
        if 'gazebo' not in self.children:
            self.children['gazebo'] = Gazebo('none', dict(lighting=None))
        self.children['gazebo']._add_child_element('lighting', value)

    def to_sdf(self):
        from ..sdf import create_sdf_element

        obj = create_sdf_element('material')
        if self._mode == 'color':
            obj.diffuse = self.color.rgba
            obj.ambient = self.color.rgba
        else:
            obj.shader = create_sdf_element('shader')
            obj.shader = self.texture.to_sdf()
        return obj
