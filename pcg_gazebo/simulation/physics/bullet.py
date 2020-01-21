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

from __future__ import print_function
from .physics import Physics
from ...parsers.sdf import create_sdf_element


class Bullet(Physics):
    def __init__(self, max_step_size=0.001, real_time_factor=1,
                 real_time_update_rate=1000, max_contacts=20,
                 min_step_size=0.0001, iters=50, sor=1.3, cfm=0, erp=0.2,
                 contact_surface_layer=0.001, split_impulse=True,
                 split_impulse_penetration_threshold=-0.01):
        Physics.__init__(
            self,
            max_step_size=max_step_size,
            real_time_factor=real_time_factor,
            real_time_update_rate=real_time_update_rate,
            max_contacts=max_contacts,
            engine='bullet')

        assert min_step_size > 0
        assert iters > 0
        assert sor > 0
        assert cfm >= 0
        assert erp >= 0
        assert contact_surface_layer > 0
        assert isinstance(split_impulse, bool)
        assert isinstance(split_impulse_penetration_threshold, float)

        self._properties['min_step_size'] = min_step_size
        self._properties['iters'] = iters
        self._properties['sor'] = sor
        self._properties['type'] = 'sequential_impulse'
        self._properties['cfm'] = cfm
        self._properties['erp'] = erp
        self._properties['contact_surface_layer'] = contact_surface_layer
        self._properties['split_impulse'] = split_impulse
        self._properties['split_impulse_penetration_threshold'] = \
            split_impulse_penetration_threshold

        self._description['min_step_size'] = 'The time duration which ' \
            'advances with each iteration of the dynamics engine, this has' \
            ' to be no bigger than max_step_size under physics block. If ' \
            'left unspecified, min_step_size defaults to max_step_size'
        self._description['iters'] = 'Number of iterations for each step.' \
            ' A higher number produces greater accuracy at a performance cost.'
        self._description['sor'] = 'Set the successive over-relaxation' \
            ' parameter.'
        self._description['type'] = 'One of the following types: ' \
            'sequential_impulse only'
        self._description['cfm'] = 'Constraint force mixing parameter'
        self._description['erp'] = 'Error reduction parameter'
        self._description['contact_surface_layer'] = 'The depth of the' \
            ' surface layer around all geometry objects. Contacts are ' \
            'allowed to sink into the surface layer up to the given depth' \
            ' before coming to rest. The default value is zero. Increasing ' \
            'this to some small value (e.g. 0.001) can help prevent ' \
            'jittering problems due to contacts being repeatedly made ' \
            'and broken.'
        self._description['split_impulse'] = 'Similar to ODE max_vel ' \
            'implementation'
        self._description['split_impulse_penetration_threshold'] = 'Similar' \
            'to ODE max_vel implementation'

    @property
    def min_step_size(self):
        return self._properties['min_step_size']

    @min_step_size.setter
    def min_step_size(self, value):
        assert value > 0
        self._properties['min_step_size'] = value

    @property
    def iters(self):
        return self._properties['iters']

    @iters.setter
    def iters(self, value):
        assert value > 0
        self._properties['iters'] = value

    @property
    def sor(self):
        return self._properties['sor']

    @sor.setter
    def sor(self, value):
        assert value > 0
        self._properties['sor'] = value

    @property
    def type(self):
        return self._properties['type']

    @property
    def cfm(self):
        return self._properties['cfm']

    @cfm.setter
    def cfm(self, value):
        assert value >= 0
        self._properties['cfm'] = value

    @property
    def erp(self):
        return self._properties['erp']

    @erp.setter
    def erp(self, value):
        assert value >= 0
        self._properties['erp'] = value

    @property
    def contact_surface_layer(self):
        return self._properties['contact_surface_layer']

    @contact_surface_layer.setter
    def contact_surface_layer(self, value):
        assert value > 0
        self._properties['contact_surface_layer'] = value

    @property
    def split_impulse(self):
        return self._properties['split_impulse']

    @split_impulse.setter
    def split_impulse(self, value):
        assert value > 0
        self._properties['split_impulse'] = value

    @property
    def split_impulse_penetration_threshold(self):
        return self._properties['split_impulse_penetration_threshold']

    @split_impulse_penetration_threshold.setter
    def split_impulse_penetration_threshold(self, value):
        assert value > 0
        self._properties['split_impulse_penetration_threshold'] = value

    def to_sdf(self, type='physics', with_default_ground_plane=True,
               with_default_sun=True):
        assert type in ['physics', 'world', 'sdf']

        # Create SDF Physics element
        physics = create_sdf_element('physics')
        physics.reset(mode='bullet', with_optional_elements=True)
        # Set all global physics parameters
        physics.name = self._properties['name']
        physics.type = self._properties['engine']
        physics.max_step_size = self._properties['max_step_size']
        physics.real_time_factor = self._properties['real_time_factor']
        physics.real_time_update_rate = \
            self._properties['real_time_update_rate']
        physics.max_contacts = self._properties['max_contacts']
        # Set Bullet solver parameters
        physics.bullet.solver.min_step_size = self._properties['min_step_size']
        physics.bullet.solver.iters = self._properties['iters']
        physics.bullet.solver.sor = self._properties['sor']
        physics.bullet.solver.type = self._properties['type']
        # Set Bullet constraint parameters
        physics.bullet.constraints.cfm = self._properties['cfm']
        physics.bullet.constraints.erp = self._properties['erp']
        physics.bullet.constraints.contact_surface_layer = self._properties[
            'contact_surface_layer']
        physics.bullet.constraints.split_impulse = \
            self._properties['split_impulse']
        physics.bullet.constraints.split_impulse_penetration_threshold = \
            self._properties[
                'split_impulse_penetration_threshold']

        if type == 'physics':
            return physics

        # Create a parent element of type world to include the physics
        # configuration
        world = create_sdf_element('world')
        world.physics = physics

        if with_default_ground_plane:
            ground_plane = create_sdf_element('include')
            ground_plane.uri = 'model://ground_plane'
            world.add_include(None, ground_plane)

        if with_default_sun:
            sun = create_sdf_element('include')
            sun.uri = 'model://sun'
            world.add_include(None, sun)

        if type == 'world':
            return world

        sdf = create_sdf_element('sdf')
        sdf.reset('world')
        sdf.world = world
        return sdf
