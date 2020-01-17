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
from ...parsers.sdf import create_sdf_element


class Physics(object):
    _PHYSICS_ENGINES = ['ode', 'bullet', 'simbody', 'dart']

    def __init__(self, max_step_size=0.001, real_time_factor=1,
                 real_time_update_rate=1000, max_contacts=20, engine='ode',
                 name='default_physics', default=False):
        assert max_step_size > 0, 'Max. step size must' \
            ' be greater than zero'
        assert real_time_factor > 0, 'Real time factor' \
            ' must be greater than zero'
        assert real_time_update_rate > 0, 'Real time update' \
            ' rate must be greater than zero'
        assert max_contacts > 0, 'Number of max.' \
            ' contancts must be greater than zero'
        assert engine in self._PHYSICS_ENGINES, 'Physics' \
            ' engine name must belong to list {}'.format(
                self._PHYSICS_ENGINES)
        assert isinstance(name, str), 'Physics name must be a string'
        assert len(name) > 0, 'Physics name input cannot be empty'

        self._properties = dict(
            max_step_size=max_step_size,
            real_time_factor=real_time_factor,
            real_time_update_rate=real_time_update_rate,
            max_contacts=max_contacts,
            engine=engine,
            name=name,
            default=default,
        )

        # Add parameter description for help function
        self._description = dict()
        self._description['max_step_size'] = 'Maximum time step size at' \
            ' which every system in simulation can interact with the states' \
            ' of the world'
        self._description['real_time_factor'] = 'Target simulation speedup' \
            'factor, defined by ratio of simulation time to real-time'
        self._description['real_time_update_rate'] = 'Rate at which to ' \
            'update the physics engine (UpdatePhysics calls per real-time ' \
            'second)'
        self._description['max_contacts'] = 'Maximum number of contacts' \
            'allowed between two entities. This value can be over ridden by' \
            ' a max_contacts element in a collision element'
        self._description['engine'] = 'The type of the dynamics engine.' \
            ' Current options are ode, bullet, simbody and dart. Defaults to' \
            ' ode if left unspecified.'
        self._description['name'] = 'The name of this set of physics ' \
            'parameters'
        self._description['default'] = 'If true, this physics element is set' \
            ' as the default physics profile for the world. If multiple ' \
            'default physics elements exist, the first element marked as ' \
            'default is chosen. If no default physics element exists, the' \
            ' first physics element is chosen.'

    @property
    def name(self):
        return self._properties['name']

    @name.setter
    def name(self, value):
        assert isinstance(value, str)
        assert len(value) > 0
        self._properties['name'] = value

    @property
    def default(self):
        return self._properties['default']

    @default.setter
    def default(self, value):
        assert isinstance(value, bool) or value in [0, 1], \
            'Default must be a boolean flag'
        self._properties['default'] = bool(value)

    @property
    def engine(self):
        return self._properties['engine']

    @engine.setter
    def engine(self, value):
        assert value in self._PHYSICS_ENGINES, \
            'Invalid physics engine, options={}, received={}'.format(
                self._PHYSICS_ENGINES, value)
        self._properties['engine'] = value

    @property
    def max_step_size(self):
        return self._properties['max_step_size']

    @max_step_size.setter
    def max_step_size(self, value):
        assert value > 0
        self._properties['max_step_size'] = value

    @property
    def real_time_factor(self):
        return self._properties['real_time_factor']

    @real_time_factor.setter
    def real_time_factor(self, value):
        assert value > 0
        self._properties['real_time_factor'] = value

    @property
    def real_time_update_rate(self):
        return self._properties['real_time_update_rate']

    @real_time_update_rate.setter
    def real_time_update_rate(self, value):
        assert value > 0
        self._properties['real_time_update_rate'] = value

    @property
    def max_contacts(self):
        return self._properties['max_contacts']

    @max_contacts.setter
    def max_contacts(self, value):
        assert value > 0
        self._properties['max_contacts'] = value

    def print_description(self, param_name):
        if param_name not in self._description:
            print(
                'No parameter description'
                ' available for {}'.format(param_name))
            return

        print(param_name)
        print('  Description: ' + self._description[param_name])
        print('  Current value: {}'.format(self._properties[param_name]))

    def get_parameter_names(self):
        return [k for k in self._properties.keys()]

    def get_parameter(self, name):
        if name in self._properties:
            return self._properties[name]
        return None

    def to_sdf(self, type='physics', with_default_ground_plane=True,
               with_default_sun=True):
        assert type in ['physics', 'world', 'sdf']

        # Create SDF Physics element
        physics = create_sdf_element('physics')
        # Set all the class parameters
        physics.name = self._properties['name']
        physics.type = self._properties['engine']
        physics.max_step_size = self._properties['max_step_size']
        physics.real_time_factor = self._properties['real_time_factor']
        physics.real_time_update_rate = \
            self._properties['real_time_update_rate']
        physics.max_contacts = self._properties['max_contacts']

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

    @staticmethod
    def from_sdf(sdf):
        if sdf.type == 'ode':
            from . import ODE
            physics = ODE()
        elif sdf.type == 'bullet':
            from . import Bullet
            physics = Bullet()
        elif sdf.type == 'simbody':
            from . import Simbody
            physics = Simbody()
        else:
            raise NotImplementedError()

        physics.name = sdf.name
        physics.engine = sdf.type
        physics.default = int(sdf.default)

        if sdf.max_step_size is not None:
            physics.max_step_size = sdf.max_step_size.value
        if sdf.real_time_factor is not None:
            physics.real_time_factor = sdf.real_time_factor.value
        if sdf.real_time_update_rate is not None:
            physics.real_time_update_rate = sdf.real_time_update_rate.value
        if sdf.max_contacts is not None:
            physics.max_contacts = sdf.max_contacts.value

        if sdf.type == 'ode':
            if sdf.ode is not None:
                if sdf.ode.solver is not None:
                    solver_param_names = [
                        'type',
                        'iters',
                        'use_dynamic_moi_rescaling',
                        'precon_iters',
                        'sor',
                        'min_step_size',
                        'friction_model']

                    for param_name in solver_param_names:
                        if getattr(sdf.ode.solver, param_name) is not None:
                            setattr(
                                physics, param_name, getattr(
                                    sdf.ode.solver, param_name).value)
                if sdf.ode.constraints is not None:
                    const_param_names = [
                        'cfm',
                        'erp',
                        'contact_surface_layer',
                        'contact_max_correcting_vel']

                    for param_name in const_param_names:
                        if getattr(
                                sdf.ode.constraints,
                                param_name) is not None:
                            setattr(
                                physics, param_name, getattr(
                                    sdf.ode.constraints, param_name).value)
        elif sdf.type == 'bullet':
            if sdf.bullet is not None:
                if sdf.bullet.solver is not None:
                    solver_param_names = [
                        'type',
                        'iters',
                        'sor',
                        'min_step_size']

                    for param_name in solver_param_names:
                        if getattr(sdf.bullet.solver, param_name) is not None:
                            setattr(
                                physics, param_name, getattr(
                                    sdf.bullet.solver, param_name).value)
                if sdf.bullet.constraints is not None:
                    const_param_names = [
                        'cfm',
                        'erp',
                        'contact_surface_layer',
                        'split_impulse',
                        'split_impulse_penetration_threshold']

                    for param_name in const_param_names:
                        if getattr(
                                sdf.bullet.constraints,
                                param_name) is not None:
                            setattr(
                                physics, param_name, getattr(
                                    sdf.bullet.constraints, param_name).value)
        elif sdf.type == 'simbody':
            if sdf.simbody is not None:
                param_names = [
                    'min_step_size',
                    'accuracy',
                    'max_transient_velocity']
                for param_name in param_names:
                    if getattr(sdf.simbody, param_name) is not None:
                        setattr(
                            physics, param_name, getattr(
                                sdf.simbody, param_name).value)
                if sdf.simbody.contact is not None:
                    param_names = [
                        'stiffness',
                        'dissipation',
                        'plastic_coef_restitution',
                        'plastic_impact_velocity',
                        'static_friction',
                        'dynamic_friction',
                        'viscous_friction',
                        'override_impact_capture_velocity',
                        'override_stiction_transition_velocity']

                    for param_name in param_names:
                        if getattr(
                                sdf.simbody.contact,
                                param_name) is not None:
                            setattr(
                                physics, param_name, getattr(
                                    sdf.simbody.contact, param_name).value)

        return physics
