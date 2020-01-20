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
from ..parsers.sdf import create_sdf_element
from .properties import Collision, Visual


class Plane(Link):
    def __init__(self, name='plane', normal=[0, 0, 1], size=[1, 1]):
        assert isinstance(normal, list)
        assert len(normal) == 3
        assert sum(normal) == 1

        assert isinstance(size, list)
        assert len(size) == 2

        Link.__init__(self, name=name)
        # Disable collision per default
        self.disable_collision()
        # Enable visual element
        self.enable_visual()
        # Properties
        self._size = size
        self._normal = normal

        self._collisions = [Collision()]
        self._visuals = [Visual()]

        self.static = True
        self._generate_collision = False

        if self._inertial:
            self.update_inertial()
        self.update_collision()
        self.update_visual()

    @property
    def normal(self):
        return self._normal

    @normal.setter
    def normal(self, value):
        assert isinstance(value, list)
        assert len(value) == 3
        assert sum(value) == 1
        self._normal = value
        self.update_collision()
        self.update_visual()

    @property
    def size(self):
        return self._size

    @size.setter
    def size(self, value):
        assert isinstance(value, list)
        assert len(value) == 2
        self._size = value
        self.update_collision()
        self.update_visual()

    @property
    def collision(self):
        return self._collisions[0]

    @property
    def visual(self):
        return self._visuals[0]

    def to_sdf(self, type='model', name='plane', sdf_version='1.6',
               resource_prefix='', model_folder=None, copy_resources=False):
        assert type in ['plane', 'geometry', 'collision', 'visual', 'link',
                        'model', 'sdf'], \
            'Invalid type of the output SDF structure'
        if type in ['collision', 'visual', 'link', 'model']:
            assert isinstance(name, str), 'Name must be a string'
            assert len(name) > 0, 'Name string cannot be empty'
        # Create a plane entity
        plane = create_sdf_element('plane')
        # Set the plane radius
        plane.size = self._size
        plane.normal = self._normal

        if type == 'plane':
            return self._collisions[0].geometry.to_sdf().plane

        if type == 'geometry':
            return self._collisions[0].geometry.to_sdf()

        return Link.to_sdf(self, type, name, sdf_version, resource_prefix,
                           model_folder, copy_resources)

    def update_collision(self):
        self._collisions[0].set_geometry(
            'plane',
            dict(
                size=self._size,
                normal=self._normal))

    def update_visual(self):
        self._visuals[0].set_geometry(
            'plane',
            dict(
                size=self._size,
                normal=self._normal))
