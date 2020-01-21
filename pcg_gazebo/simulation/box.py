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
from .properties import Inertial, Collision, Visual
import collections


class Box(Link):
    """Class derived from `pcg_gazebo.simulation.Link` to
    describe a box-shaped link or single-link model.

    > *Input arguments*

    * `name` (*type:* `str`, *default:* `box`): Name of the object
    * `size` (*type:* `list`, *default:* `[1, 1, 1]`): Vector with
    width, length and height of the box,
    """

    def __init__(self, name='box', size=[1, 1, 1]):
        Link.__init__(self, name=name)
        self.enable_collision()
        self.enable_visual()
        # Properties
        self._collisions = [Collision()]
        self._visuals = [Visual()]

        assert isinstance(size, collections.Iterable), \
            'Input size vector must be a list'
        assert len(size) == 3, 'Size vector must be have 3 elements'

        for item in size:
            assert isinstance(item, float) or isinstance(item, int), \
                'Every item in the size vector must be'\
                ' either a float or an integer'
            assert item > 0, 'Every item in the size' \
                ' vector must be greater than zero'

        self._size = size
        self.update_collision()
        self.update_visual()

    @property
    def size(self):
        """List of `float`: Size of the box as `[width, length, height]`"""
        return self._size

    @size.setter
    def size(self, vec):
        assert isinstance(vec, list)
        assert len(vec) == 3
        for item in vec:
            assert isinstance(item, float) or isinstance(item, int)
            assert item > 0
        self._size = vec
        if self._inertial:
            self.update_inertial()
        self.update_collision()
        self.update_visual()

    @property
    def collision(self):
        """`pcg_gazebo.simulation.properties.Collision`:
        Return single box-shaped collision model.
        """
        return self._collisions[0]

    @property
    def visual(self):
        """`pcg_gazebo.simulation.properties.Visual`:
        Return single box-shaped visual model.
        """
        return self._visuals[0]

    def to_sdf(self, type='model', name='box', sdf_version='1.6',
               resource_prefix='', model_folder=None, copy_resources=False):
        """Convert object to an SDF element. The object can be converted
        to different SDF elements according to the `type` input

        * `box`: SDF box element
        * `geometry`: SDF geometry element with nested element
        * `collision`: SDF collision element
        * `visual`: SDF visual element
        * `link`: SDF link element with collision and visual properties
        * `model`: single-link SDF model element
        * `sdf`: SDF file format with a nested model element.

        > *Input arguments*

        * `type` (*type:* `str`): Type of output SDF element, options are
        `collision`, `visual`, `link`, `model`, `sdf`.
        * `name` (*type:* `str`, *default:* `model`): Name of the output object
        * `sdf_version` (*type:* `str`, *default:* `1.6`): Version
        of the output SDF element

        > *Returns*

        `pcg_gazebo.parsers.types.XMLBase`: SDF element instance.
        """
        assert type in ['box', 'geometry', 'collision', 'visual', 'link',
                        'model', 'sdf'], 'Invalid SDF element type'
        if type in ['collision', 'visual', 'link', 'model']:
            assert isinstance(name, str), 'Type input must be a string'
            assert len(name) > 0, 'Type input cannot be an empty string'

        if type == 'box':
            return self._collisions[0].geometry.to_sdf().box

        if type == 'geometry':
            return self._collisions[0].geometry.to_sdf()

        return Link.to_sdf(self, type, name, sdf_version, resource_prefix,
                           model_folder, copy_resources)

    def add_inertial(self, mass):
        """Initialize mass and moments of inertia for box model.

        > *Input arguments*

        * `mass` (*type:* `float`): Mass in kilograms
        """
        assert isinstance(mass, float) or isinstance(mass, int)
        assert mass > 0

        self.update_inertial(mass)

    def update_inertial(self, mass=None):
        """Update mass and moments of inertia for box model.

        > *Input arguments*

        * `mass` (*type:* `float`): Mass in kilograms
        """
        if self._inertial is not None and mass is None:
            mass = self._inertial.mass

        self._inertial = Inertial.create_cuboid_inertia(
            mass,
            self._size[0],
            self._size[1],
            self._size[2])

    def update_collision(self):
        """Update collision model according to the current
        `size`.
        """
        self._collisions[0].set_geometry(
            'box',
            dict(size=self._size))

    def update_visual(self):
        """Update visual model according to the current
        `size`.
        """
        self._visuals[0].set_geometry(
            'box',
            dict(size=self._size))
