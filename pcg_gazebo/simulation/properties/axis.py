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
import collections


class Axis(object):
    def __init__(
            self,
            xyz=[
                0,
                0,
                1],
            lower_limit=-1e16,
            upper_limit=1e16,
            velocity_limit=-1,
            effort_limit=-1,
            damping=0,
            friction=0,
            spring_reference=0,
            spring_stiffness=0,
            use_parent_model_frame=False):
        self._xyz = [0, 0, 1]
        self._lower_limit = lower_limit
        self._upper_limit = upper_limit
        self._velocity_limit = velocity_limit
        self._effort_limit = effort_limit
        self._damping = damping
        self._friction = friction
        self._spring_reference = spring_reference
        self._spring_stiffness = spring_stiffness
        self._use_parent_model_frame = use_parent_model_frame

        self.set_axis(xyz)
        self.set_limits(lower_limit, upper_limit, velocity_limit, effort_limit)
        self.set_dynamics(
            damping,
            friction,
            spring_reference,
            spring_stiffness)

    @property
    def xyz(self):
        return self._xyz

    @property
    def lower_limit(self):
        return self._lower_limit

    @property
    def upper_limit(self):
        return self._upper_limit

    @property
    def velocity_limit(self):
        return self._velocity_limit

    @property
    def effort_limit(self):
        return self._effort_limit

    @property
    def damping(self):
        return self._damping

    @property
    def friction(self):
        return self._friction

    @property
    def spring_reference(self):
        return self._spring_reference

    @property
    def spring_stiffness(self):
        return self._spring_stiffness

    @property
    def use_parent_model_frame(self):
        return self._use_parent_model_frame

    def set_axis(self, vec):
        assert isinstance(vec, collections.Iterable), \
            'Input vector must be a list or an array'
        vec = list(vec)
        assert len(vec) == 3, 'Input axis vector must have three elements'
        for elem in vec:
            assert isinstance(elem, float) or isinstance(elem, int), \
                'Each element of the input vector' \
                ' must be either a float or an integer'
        assert sum(vec) == 1, 'Input vector must be an unit vector'
        self._xyz = vec

    def set_limits(self, lower=-1e16, upper=1e16, velocity=-1, effort=-1):
        assert lower <= upper, 'Lower joint limit must be less ' \
            'or equal to the upper joint limit'
        if velocity != -1:
            assert velocity >= 0, \
                'Velocity limit must be greater or equal to zero'
        if effort != -1:
            assert effort >= 0, \
                'Effort limit must be greater or equal to zero'
        self._lower_limit = lower
        self._upper_limit = upper
        self._velocity_limit = velocity
        self._effort_limit = effort

    def set_dynamics(
            self,
            damping=0,
            friction=0,
            spring_reference=0,
            spring_stiffness=0):
        assert damping >= 0, \
            'Damping coefficient must be greater or equal to zero'
        assert friction >= 0, \
            'Friction coefficient must be greater or equal to zero'
        assert spring_reference >= 0, \
            'Spring reference must be greater or equal to zero'
        assert spring_stiffness >= 0, \
            'Spring stiffness must be greater or equal to zero'
        self._damping = damping
        self._friction = friction
        self._spring_reference = spring_reference
        self._spring_stiffness

    def set_use_parent_model_frame(self, flag=False):
        self._use_parent_model_frame = flag

    def to_sdf(self, is_axis_2=False):
        from ...parsers.sdf import create_sdf_element

        axis = create_sdf_element(
            'axis') if not is_axis_2 else create_sdf_element('axis2')
        axis.xyz = self._xyz
        axis.limit = dict(
            lower=self._lower_limit,
            upper=self._upper_limit,
            velocity=self._velocity_limit,
            effort=self._effort_limit)
        axis.dynamics = dict(
            damping=self._damping,
            friction=self._friction,
            spring_reference=self._spring_reference,
            spring_stiffness=self._spring_stiffness)
        axis.use_parent_model_frame = self._use_parent_model_frame
        return axis

    @staticmethod
    def from_sdf(sdf):
        assert sdf is not None, 'Input SDF element is invalid'
        assert sdf.xml_element_name in [
            'axis', 'axis2'], 'Invalid SDF axis element'

        axis = Axis()
        if sdf.xyz is not None:
            axis.set_axis(sdf.xyz.value)
        if sdf.limit is not None:
            axis.set_limits(
                sdf.limit.lower.value,
                sdf.limit.upper.value,
                -1 if sdf.limit.velocity is None
                else sdf.limit.velocity.value,
                -1 if sdf.limit.effort is None
                else sdf.limit.effort.value
            )
        if sdf.dynamics is not None:
            axis.set_dynamics(
                0 if sdf.dynamics.damping is None
                else sdf.dynamics.damping.value,
                0 if sdf.dynamics.friction is None
                else sdf.dynamics.friction.value,
                0 if sdf.dynamics.spring_reference is None
                else sdf.dynamics.spring_reference.value,
                0 if sdf.dynamics.spring_stiffness is None
                else sdf.dynamics.spring_stiffness.value)
        return axis
    # TODO Add ODE and Bullet specific physics parameters
