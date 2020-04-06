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

import os
from ... import random
from ...parsers.sdf import create_sdf_element


class Material(object):
    _GAZEBO_MATERIALS = [
        'Gazebo/Grey',
        'Gazebo/DarkGrey',
        'Gazebo/White',
        'Gazebo/FlatBlack',
        'Gazebo/Black',
        'Gazebo/Red',
        'Gazebo/RedBright',
        'Gazebo/Green',
        'Gazebo/Blue',
        'Gazebo/SkyBlue',
        'Gazebo/Yellow',
        'Gazebo/ZincYellow',
        'Gazebo/DarkYellow',
        'Gazebo/Purple',
        'Gazebo/Turquoise',
        'Gazebo/Orange',
        'Gazebo/Indigo',
        'Gazebo/WhiteGlow',
        'Gazebo/RedGlow',
        'Gazebo/GreenGlow',
        'Gazebo/BlueGlow',
        'Gazebo/YellowGlow',
        'Gazebo/PurpleGlow',
        'Gazebo/TurquoiseGlow',
        'Gazebo/TurquoiseGlowOutline',
        'Gazebo/RedTransparentOverlay',
        'Gazebo/BlueTransparentOverlay',
        'Gazebo/GreenTransparentOverlay',
        'Gazebo/OrangeTransparentOverlay',
        'Gazebo/DarkOrangeTransparentOverlay',
        'Gazebo/RedTransparent',
        'Gazebo/GreenTransparent',
        'Gazebo/BlueTransparent',
        'Gazebo/DarkMagentaTransparent',
        'Gazebo/GreyTransparent',
        'Gazebo/BlackTransparent',
        'Gazebo/YellowTransparent',
        'Gazebo/OrangeTransparent',
        'Gazebo/WoodFloor',
        'Gazebo/CeilingTiled',
        'Gazebo/PaintedWall',
        'Gazebo/BuildingFrame',
        'Gazebo/Runway',
        'Gazebo/Grass'
    ]

    def __init__(self):
        self._xkcd_colors = self.get_xkcd_colors_list()
        self._gazebo_material_script_default_uri = \
            'file://media/materials/scripts/gazebo.material'

    @property
    def xkcd_colors(self):
        return self._xkcd_colors

    @property
    def gazebo_materials(self):
        return self._GAZEBO_MATERIALS

    def get_xkcd_material_as_sdf(self, name):
        if name not in self._xkcd_colors:
            print('Color {} does not exist')
            return None
        obj = create_sdf_element('material')
        obj.ambient = [i / 255. for i in self._xkcd_colors[name]] + [1]
        obj.diffuse = [i / 255. for i in self._xkcd_colors[name]] + [1]
        return obj

    def get_color_material(self, r=None, g=None, b=None, a=1):
        obj = create_sdf_element('material')
        if r is None or r < 0:
            r = random.rand()
        if g is None or g < 0:
            g = random.rand()
        if b is None or b < 0:
            b = random.rand()
        obj.ambient = [i for i in [r, g, b]] + [a]
        obj.diffuse = [i for i in [r, g, b]] + [a]
        return obj

    def get_random_xkcd_material_as_sdf(self):
        k = list(self._xkcd_colors.keys())
        color_name = k[random.choice(range(len(k)))]
        return self.get_xkcd_material_as_sdf(color_name)

    @staticmethod
    def get_gazebo_material_as_sdf(name):
        obj = create_sdf_element('material')
        obj.script = create_sdf_element('script')
        obj.script.name = name
        obj.script.add_uri('file://media/materials/scripts/gazebo.material')
        return obj

    @staticmethod
    def get_script_material_as_sdf(name, uri):
        obj = create_sdf_element('material')
        obj.script = create_sdf_element('script')
        obj.script.name = name
        obj.script.add_uri(uri)
        return obj

    @staticmethod
    def get_gazebo_materials_list():
        return Material._GAZEBO_MATERIALS

    @staticmethod
    def get_xkcd_colors_list():
        xkcd_filename = os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            'resources', 'xkcd_rgb.txt')

        colors = dict()
        with open(xkcd_filename, 'r') as xkcd_file:
            for line in xkcd_file:
                if 'License:' in line:
                    continue
                color_name = line.split('#')[0].replace(
                    ' ', '_').replace('\t', '')
                hex = line.split('#')[1]
                colors[color_name] = tuple(
                    int(hex[i:i + 2], 16) for i in (0, 2, 4))
        return colors
