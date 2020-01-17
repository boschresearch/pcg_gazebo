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

from .link import Link
from .properties import Collision, Visual


class Polyline(Link):
    def __init__(self, name='polyline', height=1, points=list()):
        assert isinstance(points, list)
        assert isinstance(height, float) or isinstance(height, int)
        # Call super class constructor
        Link.__init__(self, name=name)
        # Disable collision per default
        self.disable_collision()
        # Enable visual element
        self.enable_visual()
        # Properties
        self._height = height
        self._points = points
        self._collisions = [Collision()]
        self._visuals = [Visual()]

        self.static = True
        self._generate_collision = False

        if self._inertial:
            self.update_inertial()
        self.update_collision()
        self.update_visual()

    @property
    def height(self):
        return self._height

    @height.setter
    def height(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        self._height = value
        self.update_collision()
        self.update_visual()

    @property
    def points(self):
        return self._points

    @property
    def collision(self):
        return self._collisions[0]

    @property
    def visual(self):
        return self._visuals[0]

    def add_point(self, x, y):
        assert isinstance(x, float) or isinstance(x, int)
        assert isinstance(y, float) or isinstance(y, int)
        self._points.append([x, y])
        self.update_collision()
        self.update_visual()

    def to_sdf(self, type='model', name='polyline', sdf_version='1.6'):
        assert type in ['polyline', 'geometry', 'collision', 'visual',
                        'link', 'model', 'sdf'], \
            'Invalid type of the output SDF structure'
        if type in ['collision', 'visual', 'link', 'model']:
            assert isinstance(name, str), 'Name must be a string'
            assert len(name) > 0, 'Name string cannot be empty'

        if type == 'polyline':
            return self._collisions[0].geometry.polyline

        if type == 'geometry':
            return self._collisions[0].geometry

        return Link.to_sdf(self, type, name, sdf_version)

    def update_collision(self):
        self._collisions[0].set_geometry(
            'polyline',
            dict(
                height=self._height,
                point=self._points))

    def update_visual(self):
        self._visuals[0].set_geometry(
            'polyline',
            dict(
                height=self._height,
                point=self._points))
