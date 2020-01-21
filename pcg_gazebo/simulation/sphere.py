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


class Sphere(Link):
    """Class derived from `pcg_gazebo.simulation.Link` to
    describe a sphere-shaped link or single-link model.

    > *Input arguments*

    * `name` (*type:* `str`, *default:* `sphere`): Name of the object
    * `radius` (*type:* `float`, *default:* `1`): Radius of the sphere
    in meters
    """

    def __init__(self, name='sphere', radius=1):
        Link.__init__(self, name=name)
        self.enable_collision()
        self.enable_visual()
        # Properties
        self._is_hollow = False
        self._collisions = [Collision()]
        self._visuals = [Visual()]
        self.radius = radius

    @property
    def radius(self):
        """`float`: Radius of the sphere in meters"""
        return self._radius

    @radius.setter
    def radius(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        self._radius = value
        if self._inertial:
            self.update_inertial()
        self.update_collision()
        self.update_visual()

    @property
    def collision(self):
        """`pcg_gazebo.simulation.properties.Collision`:
        Return single sphere-shaped collision model.
        """
        return self._collisions[0]

    @property
    def visual(self):
        """`pcg_gazebo.simulation.properties.Visual`:
        Return single sphere-shaped visual model.
        """
        return self._visuals[0]

    def to_sdf(self, type='model', name='sphere', sdf_version='1.6',
               resource_prefix='', model_folder=None, copy_resources=False):
        """Convert object to an SDF element. The object can be converted
        to different SDF elements according to the `type` input

        * `sphere`: SDF sphere element
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
        * `sdf_version` (*type:* `str`, *default:* `1.6`): Version of
        the output SDF element

        > *Returns*

        `pcg_gazebo.parsers.types.XMLBase`: SDF element instance.
        """
        assert type in ['sphere', 'geometry', 'collision', 'visual', 'link',
                        'model', 'sdf'], \
            'Invalid type of the output SDF structure'
        if type in ['collision', 'visual', 'link', 'model']:
            assert isinstance(name, str), 'Name must be a string'
            assert len(name) > 0, 'Name string cannot be empty'

        if type == 'sphere':
            return self._collisions[0].geometry.to_sdf().sphere

        if type == 'geometry':
            return self._collisions[0].geometry.to_sdf()

        return Link.to_sdf(self, type, name, sdf_version, resource_prefix,
                           model_folder, copy_resources)

    def add_inertial(self, mass, hollow=False):
        """Initialize mass and moments of inertia for sphere model.

        > *Input arguments*

        * `mass` (*type:* `float`): Mass in kilograms
        * `hollow` (*type:* `bool`, *default:* `False`): Compute
        moments of inertia for a hollow sphere, instead of a solid one
        """
        assert isinstance(mass, float) or isinstance(mass, int)
        assert mass > 0
        self._is_hollow = hollow

        self.update_inertial(mass)

    def update_inertial(self, mass=None):
        """Update mass and moments of inertia for sphere model.

        > *Input arguments*

        * `mass` (*type:* `float`): Mass in kilograms
        """
        if self._inertial is not None and mass is None:
            mass = self._inertial.mass

        if self._is_hollow:
            self._inertial = \
                Inertial.create_hollow_sphere_inertia(
                    mass,
                    self._radius)
        else:
            self._inertial = \
                Inertial.create_solid_sphere_inertia(
                    mass,
                    self._radius)

    def update_collision(self):
        """Update collision model according to the current
        `radius`.
        """
        self._collisions[0].set_geometry(
            'sphere',
            dict(radius=self._radius))

    def update_visual(self):
        """Update visual model according to the current
        `radius`.
        """
        self._visuals[0].set_geometry(
            'sphere',
            dict(radius=self._radius))
