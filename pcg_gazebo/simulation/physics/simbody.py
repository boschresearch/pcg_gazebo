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


class Simbody(Physics):

    def __init__(self, max_step_size=0.001, real_time_factor=1,
                 real_time_update_rate=1000, max_contacts=20,
                 min_step_size=0.0001, accuracy=0.001,
                 max_transient_velocity=0.01, stiffness=1e8,
                 dissipation=100, plastic_coef_restitution=0.5,
                 plastic_impact_velocity=0.5, static_friction=0.9,
                 dynamic_friction=0.9, viscous_friction=0,
                 override_impact_capture_velocity=0.001,
                 override_stiction_transition_velocity=0.001):
        Physics.__init__(
            self,
            max_step_size=max_step_size,
            real_time_factor=real_time_factor,
            real_time_update_rate=real_time_update_rate,
            max_contacts=max_contacts,
            engine='simbody')

        assert min_step_size > 0
        assert accuracy > 0
        assert max_transient_velocity > 0
        assert stiffness >= 0
        assert dissipation >= 0
        assert plastic_coef_restitution >= 0
        assert plastic_impact_velocity >= 0
        assert static_friction >= 0
        assert dynamic_friction >= 0
        assert viscous_friction >= 0
        assert override_impact_capture_velocity >= 0
        assert override_stiction_transition_velocity >= 0

        self._properties['min_step_size'] = min_step_size
        self._properties['accuracy'] = accuracy
        self._properties['max_transient_velocity'] = max_transient_velocity
        self._properties['stiffness'] = stiffness
        self._properties['dissipation'] = dissipation
        self._properties['plastic_coef_restitution'] = plastic_coef_restitution
        self._properties['plastic_impact_velocity'] = plastic_impact_velocity
        self._properties['static_friction'] = static_friction
        self._properties['dynamic_friction'] = dynamic_friction
        self._properties['viscous_friction'] = viscous_friction
        self._properties['override_impact_capture_velocity'] = \
            override_impact_capture_velocity
        self._properties['override_stiction_transition_velocity'] = \
            override_stiction_transition_velocity

        self._description['min_step_size'] = '(Currently not used in ' \
            'simbody) The time duration which advances with each iteration' \
            ' of the dynamics engine, this has to be no bigger than ' \
            'max_step_size under physics block. If left unspecified, ' \
            'min_step_size defaults to max_step_size'
        self._description['accuracy'] = 'Roughly the relative error of the' \
            ' system. -LOG(accuracy) is roughly the number of significant ' \
            'digits.'
        self._description['max_transient_velocity'] = 'Tolerable "slip" ' \
            'velocity allowed by the solver when static friction is supposed' \
            ' to hold object in place.'
        self._description['stiffness'] = 'Default contact material stiffness' \
            ' (force/dist or torque/radian).'
        self._description['dissipation'] = 'Dissipation coefficient to be' \
            ' used in compliant contact; if not given it is ' \
            '(1-min_cor)/plastic_impact_velocity'
        self._description['plastic_coef_restitution'] = 'This is the COR to' \
            ' be used at high velocities for rigid impacts; if not given it' \
            ' is 1 - dissipation*plastic_impact_velocity'
        self._description['plastic_impact_velocity'] = 'Smallest impact ' \
            'velocity at which min COR is reached; set to zero if you want' \
            ' the min COR always to be used'
        self._description['static_friction'] = 'Static friction [mu_s]'
        self._description['dynamic_friction'] = 'Dynamic friction [mu_d]'
        self._description['viscous_friction'] = 'Viscous friction [mu_v]' \
            ' with units of 1 / velocity'
        self._description['override_impact_capture_velocity'] = 'For rigid' \
            ' impacts only, impact velocity at which COR is set to zero;' \
            ' normally inherited from global default but can be overridden' \
            ' here. Combining rule: use larger velocity'
        self._description['override_stiction_transition_velocity'] = 'This' \
            ' is the largest slip velocity at which we will consider a' \
            ' transition to stiction. Normally inherited from a global' \
            ' default setting. For a continuous friction model this is the' \
            ' velocity at which the max static friction force is reached.' \
            ' Combining rule: use larger velocity'

    @property
    def min_step_size(self):
        return self._properties['min_step_size']

    @min_step_size.setter
    def min_step_size(self, value):
        assert value > 0
        self._properties['min_step_size'] = value

    @property
    def accuracy(self):
        return self._properties['accuracy']

    @accuracy.setter
    def accuracy(self, value):
        assert value > 0
        self._properties['accuracy'] = value

    @property
    def max_transient_velocity(self):
        return self._properties['max_transient_velocity']

    @max_transient_velocity.setter
    def max_transient_velocity(self, value):
        assert value > 0
        self._properties['max_transient_velocity'] = value

    @property
    def stiffness(self):
        return self._properties['stiffness']

    @stiffness.setter
    def stiffness(self, value):
        assert value >= 0
        self._properties['stiffness'] = value

    @property
    def dissipation(self):
        return self._properties['dissipation']

    @dissipation.setter
    def dissipation(self, value):
        assert value >= 0
        self._properties['dissipation'] = value

    @property
    def plastic_coef_restitution(self):
        return self._properties['plastic_coef_restitution']

    @plastic_coef_restitution.setter
    def plastic_coef_restitution(self, value):
        assert value >= 0
        self._properties['plastic_coef_restitution'] = value

    @property
    def plastic_impact_velocity(self):
        return self._properties['plastic_impact_velocity']

    @plastic_impact_velocity.setter
    def plastic_impact_velocity(self, value):
        assert value >= 0
        self._properties['plastic_impact_velocity'] = value

    @property
    def static_friction(self):
        return self._properties['static_friction']

    @static_friction.setter
    def static_friction(self, value):
        assert value >= 0
        self._properties['static_friction'] = value

    @property
    def dynamic_friction(self):
        return self._properties['dynamic_friction']

    @dynamic_friction.setter
    def dynamic_friction(self, value):
        assert value >= 0
        self._properties['dynamic_friction'] = value

    @property
    def viscous_friction(self):
        return self._properties['viscous_friction']

    @viscous_friction.setter
    def viscous_friction(self, value):
        assert value >= 0
        self._properties['viscous_friction'] = value

    @property
    def override_impact_capture_velocity(self):
        return self._properties['override_impact_capture_velocity']

    @override_impact_capture_velocity.setter
    def override_impact_capture_velocity(self, value):
        assert value >= 0
        self._properties['override_impact_capture_velocity'] = value

    @property
    def override_stiction_transition_velocity(self):
        return self._properties['override_stiction_transition_velocity']

    @override_stiction_transition_velocity.setter
    def override_stiction_transition_velocity(self, value):
        assert value >= 0
        self._properties['override_stiction_transition_velocity'] = value

    def to_sdf(self, type='physics', with_default_ground_plane=True,
               with_default_sun=True):
        assert type in ['physics', 'world', 'sdf']

        # Create SDF Physics element
        physics = create_sdf_element('physics')
        physics.reset(mode='simbody', with_optional_elements=True)
        # Set all global physics parameters
        physics.name = self._properties['name']
        physics.type = self._properties['engine']
        physics.max_step_size = \
            self._properties['max_step_size']
        physics.real_time_factor = \
            self._properties['real_time_factor']
        physics.real_time_update_rate = \
            self._properties['real_time_update_rate']
        physics.max_contacts = self._properties['max_contacts']
        # Set Bullet solver parameters
        physics.simbody.min_step_size = self._properties['min_step_size']
        physics.simbody.accuracy = self._properties['accuracy']
        physics.simbody.max_transient_velocity = \
            self._properties['max_transient_velocity']
        # Set Bullet constraint parameters
        physics.simbody.contact.stiffness = \
            self._properties['stiffness']
        physics.simbody.contact.dissipation = \
            self._properties['dissipation']
        physics.simbody.contact.plastic_coef_restitution = \
            self._properties[
                'plastic_coef_restitution']
        physics.simbody.contact.plastic_impact_velocity = \
            self._properties[
                'plastic_impact_velocity']
        physics.simbody.contact.static_friction = \
            self._properties['static_friction']
        physics.simbody.contact.dynamic_friction = \
            self._properties['dynamic_friction']
        physics.simbody.contact.viscous_friction = \
            self._properties['viscous_friction']
        physics.simbody.contact.override_impact_capture_velocity = \
            self._properties[
                'override_impact_capture_velocity']
        physics.simbody.contact.override_stiction_transition_velocity = \
            self._properties[
                'override_stiction_transition_velocity']

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
