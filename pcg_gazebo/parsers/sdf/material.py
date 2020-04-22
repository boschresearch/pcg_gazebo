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
from .script import Script
from .shader import Shader
from .ambient import Ambient
from .diffuse import Diffuse
from .specular import Specular
from .emissive import Emissive
from .lighting import Lighting


class Material(XMLBase):
    _NAME = 'material'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        script=dict(creator=Script, default=['material'], optional=True),
        shader=dict(creator=Shader, optional=True),
        lighting=dict(creator=Lighting, optional=True),
        ambient=dict(creator=Ambient, optional=True),
        diffuse=dict(creator=Diffuse, optional=True),
        specular=dict(creator=Specular, optional=True),
        emissive=dict(creator=Emissive, optional=True)
    )

    def __init__(self):
        super(Material, self).__init__()

    @property
    def script(self):
        return self._get_child_element('script')

    @script.setter
    def script(self, value):
        self._add_child_element('script', value)

    @property
    def shader(self):
        return self._get_child_element('shader')

    @shader.setter
    def shader(self, value):
        self._add_child_element('shader', value)

    @property
    def lighting(self):
        return self._get_child_element('lighting')

    @lighting.setter
    def lighting(self, value):
        self._add_child_element('lighting', value)

    @property
    def ambient(self):
        return self._get_child_element('ambient')

    @ambient.setter
    def ambient(self, value):
        self._add_child_element('ambient', value)

    @property
    def diffuse(self):
        return self._get_child_element('diffuse')

    @diffuse.setter
    def diffuse(self, value):
        self._add_child_element('diffuse', value)

    @property
    def specular(self):
        return self._get_child_element('specular')

    @specular.setter
    def specular(self, value):
        self._add_child_element('specular', value)

    @property
    def emissive(self):
        return self._get_child_element('emissive')

    @emissive.setter
    def emissive(self, value):
        self._add_child_element('emissive', value)

    @staticmethod
    def get_gazebo_material(material_name):
        obj = Material()
        obj.script = Script()
        obj.script.name = material_name
        obj.script.add_uri('file://media/materials/scripts/gazebo.material')

        return obj

    @staticmethod
    def get_diffuse_material(r, g, b):
        if r > 1 or g > 1 or b > 1:
            r = min(1, r / 255.)
            g = min(1, g / 255.)
            b = min(1, b / 255.)
        obj = Material()
        obj.ambient = [r, g, b, 1]
        obj.diffuse = [r, g, b, 1]
        return obj
