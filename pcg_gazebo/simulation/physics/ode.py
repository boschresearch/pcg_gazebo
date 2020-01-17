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


class ODE(Physics):
    _FRICTION_MODELS = ['pyramid_model', 'box_model', 'cone_model']
    _SOLVER_TYPES = ['quick', 'world']

    def __init__(self, max_step_size=0.001, real_time_factor=1,
                 real_time_update_rate=1000, max_contacts=20,
                 min_step_size=0.0001, iters=50, sor=1.3, type='quick',
                 precon_iters=0, use_dynamic_moi_rescaling=False,
                 friction_model='pyramid_model', cfm=0, erp=0.2,
                 contact_surface_layer=0.001, contact_max_correcting_vel=100):
        Physics.__init__(
            self,
            max_step_size=max_step_size,
            real_time_factor=real_time_factor,
            real_time_update_rate=real_time_update_rate,
            max_contacts=max_contacts,
            engine='ode')

        assert min_step_size > 0
        assert iters > 0
        assert sor > 0
        assert type in self._SOLVER_TYPES, \
            'Invalid solver type, options are {}, received={}'.format(
                self._SOLVER_TYPES, type)
        assert precon_iters >= 0
        assert isinstance(use_dynamic_moi_rescaling, bool)
        assert friction_model in self._FRICTION_MODELS, \
            'Invalid friction model, options are {}, received={}'.format(
                self._FRICTION_MODELS, friction_model)
        assert cfm >= 0, 'cfm must be equal or greater than zero'
        assert erp >= 0, 'erp must be equal or greater than zero'
        assert contact_surface_layer > 0
        assert contact_max_correcting_vel > 0

        self._properties['min_step_size'] = min_step_size
        self._properties['iters'] = iters
        self._properties['sor'] = sor
        self._properties['type'] = type
        self._properties['precon_iters'] = precon_iters
        self._properties['use_dynamic_moi_rescaling'] = \
            use_dynamic_moi_rescaling
        self._properties['friction_model'] = friction_model
        self._properties['cfm'] = cfm
        self._properties['erp'] = erp
        self._properties['contact_surface_layer'] = contact_surface_layer
        self._properties['contact_max_correcting_vel'] = \
            contact_max_correcting_vel

        self._description['min_step_size'] = 'The time duration which ' \
            'advances with each iteration of the dynamics engine, this has ' \
            'to be no bigger than max_step_size under physics block. If left' \
            ' unspecified, min_step_size defaults to max_step_size'
        self._description['iters'] = 'Number of iterations for each step.' \
            ' A higher number produces greater accuracy at a performance cost.'
        self._description['sor'] = 'Set the successive over-relaxation ' \
            'parameter.'
        self._description['type'] = 'One of the following types: world, quick'
        self._description['precon_iters'] = 'Experimental parameter'
        self._description['use_dynamic_moi_rescaling'] = 'Flag to enable ' \
            'dynamic rescaling of moment of inertia in constrained ' \
            'directions'
        self._description['friction_model'] = 'Name of ODE friction model to' \
            ' use. Valid values include: pyramid_model: (default) friction' \
            ' forces limited in two directions in proportion to normal ' \
            'force. box_model: friction forces limited to constant in two' \
            ' directions. cone_model: friction force magnitude limited in ' \
            'proportion to normal force'
        self._description['cfm'] = 'Constraint force mixing parameter'
        self._description['erp'] = 'Error reduction parameter'
        self._description['contact_surface_layer'] = 'The depth of the' \
            ' surface layer around all geometry objects. Contacts are ' \
            'allowed to sink into the surface layer up to the given depth' \
            ' before coming to rest. The default value is zero. Increasing ' \
            'this to some small value (e.g. 0.001) can help prevent ' \
            'jittering problems due to contacts being repeatedly made ' \
            'and broken.'
        self._description['contact_max_correcting_vel'] = 'The maximum' \
            ' correcting velocities allowed when resolving contacts'

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

    @type.setter
    def type(self, value):
        assert value in self._SOLVER_TYPES, \
            'Invalid solver type, options={}, received={}'.format(
                self._SOLVER_TYPES, value)
        self._properties['type'] = value

    @property
    def precon_iters(self):
        return self._properties['precon_iters']

    @precon_iters.setter
    def precon_iters(self, value):
        assert value >= 0
        self._properties['precon_iters'] = value

    @property
    def use_dynamic_moi_rescaling(self):
        return self._properties['use_dynamic_moi_rescaling']

    @use_dynamic_moi_rescaling.setter
    def use_dynamic_moi_rescaling(self, value):
        assert isinstance(value, bool)
        self._properties['use_dynamic_moi_rescaling'] = value

    @property
    def friction_model(self):
        return self._properties['friction_model']

    @friction_model.setter
    def friction_model(self, value):
        assert value in self._FRICTION_MODELS
        self._properties['friction_model'] = value

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
    def contact_max_correcting_vel(self):
        return self._properties['contact_max_correcting_vel']

    @contact_max_correcting_vel.setter
    def contact_max_correcting_vel(self, value):
        assert value > 0
        self._properties['contact_max_correcting_vel'] = value

    def to_sdf(self, type='physics', with_default_ground_plane=True,
               with_default_sun=True):
        assert type in ['physics', 'world', 'sdf']

        # Create SDF Physics element
        physics = create_sdf_element('physics')
        physics.reset(mode='ode', with_optional_elements=True)
        # Set all global physics parameters
        physics.name = self._properties['name']
        physics.type = self._properties['engine']
        physics.max_step_size = self._properties['max_step_size']
        physics.real_time_factor = self._properties['real_time_factor']
        physics.real_time_update_rate = \
            self._properties['real_time_update_rate']
        physics.max_contacts = self._properties['max_contacts']
        # Set ODE solver parameters
        physics.ode.solver.type = self._properties['type']
        physics.ode.solver.iters = self._properties['iters']
        physics.ode.solver.use_dynamic_moi_rescaling = self._properties[
            'use_dynamic_moi_rescaling']
        physics.ode.solver.precon_iters = self._properties['precon_iters']
        physics.ode.solver.sor = self._properties['sor']
        physics.ode.solver.min_step_size = self._properties['min_step_size']
        physics.ode.solver.friction_model = self._properties['friction_model']
        # Set ODE constraint parameters
        physics.ode.constraints.cfm = self._properties['cfm']
        physics.ode.constraints.erp = self._properties['erp']
        physics.ode.constraints.contact_surface_layer = \
            self._properties['contact_surface_layer']
        physics.ode.constraints.contact_max_correcting_vel = self._properties[
            'contact_max_correcting_vel']

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
