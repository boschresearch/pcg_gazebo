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
import random
from ..types import XMLBase
from .box import Box
from .empty import Empty
from .image import Image
from .cylinder import Cylinder
from .sphere import Sphere
from .mesh import Mesh
from .plane import Plane
from .polyline import Polyline


class Geometry(XMLBase):
    _NAME = 'geometry'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        empty=dict(creator=Empty, mode='empty', n_elems=1),
        box=dict(creator=Box, mode='box', n_elems=1),
        image=dict(
            creator=Image, mode='image', default=['geometry'], n_elems=1),
        cylinder=dict(creator=Cylinder, mode='cylinder', n_elems=1),
        sphere=dict(creator=Sphere, mode='sphere', n_elems=1),
        plane=dict(creator=Plane, mode='plane', n_elems=1),
        mesh=dict(creator=Mesh, mode='mesh', n_elems=1),
        polyline=dict(creator=Polyline, mode='polyline', n_elems=1)
    )

    _MODES = [
        'empty',
        'box',
        'image',
        'cylinder',
        'sphere',
        'plane',
        'mesh',
        'polyline']

    def __init__(self, mode='empty'):
        XMLBase.__init__(self)
        self.reset(mode=mode)

    @property
    def empty(self):
        return self._get_child_element('empty')

    @empty.setter
    def empty(self, value):
        self.reset(mode='empty')
        self._add_child_element('empty', value)

    @property
    def box(self):
        return self._get_child_element('box')

    @box.setter
    def box(self, value):
        self.reset(mode='box')
        self._add_child_element('box', value)

    @property
    def image(self):
        return self._get_child_element('image')

    @image.setter
    def image(self, value):
        self.reset(mode='image')
        self._add_child_element('image', value)

    @property
    def cylinder(self):
        return self._get_child_element('cylinder')

    @cylinder.setter
    def cylinder(self, value):
        self.reset(mode='cylinder')
        self._add_child_element('cylinder', value)

    @property
    def sphere(self):
        return self._get_child_element('sphere')

    @sphere.setter
    def sphere(self, value):
        self.reset(mode='sphere')
        self._add_child_element('sphere', value)

    @property
    def plane(self):
        return self._get_child_element('plane')

    @plane.setter
    def plane(self, value):
        self.reset(mode='plane')
        self._add_child_element('plane', value)

    @property
    def mesh(self):
        return self._get_child_element('mesh')

    @mesh.setter
    def mesh(self, value):
        self.reset(mode='mesh')
        self._add_child_element('mesh', value)

    @property
    def polyline(self):
        return self._get_child_element('polyline')

    @polyline.setter
    def polyline(self, value):
        self.reset(mode='polyline')
        self._add_child_element('polyline', value)

    def random(self):
        try:
            self.reset(mode=random.choice(self._MODES))
            getattr(self, self._mode).random()
        except NotImplementedError:
            pass
