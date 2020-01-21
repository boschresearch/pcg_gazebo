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

from ..types import XMLBase, XMLString
from .stiffness import Stiffness
from .dissipation import Dissipation
from .plastic_coef_restitution import PlasticCoefRestitution
from .plastic_impact_velocity import PlasticImpactVelocity
from .static_friction import StaticFriction
from .dynamic_friction import DynamicFriction
from .viscous_friction import ViscousFriction
from .override_impact_capture_velocity import OverrideImpactCaptureVelocity
from .override_stiction_transition_velocity import \
    OverrideStictionTransitionVelocity
from .ode import ODE
from .bullet import Bullet
from .topic import Topic
from .collide_bitmask import CollideBitmask
from .collide_without_contact import CollideWithoutContact
from .collide_without_contact_bitmask import CollideWithoutContactBitmask
from .category_bitmask import CategoryBitmask
from .poissons_ratio import PoissonsRatio
from .elastic_modulus import ElasticModulus


class ContactCollisionName(XMLString):
    _NAME = 'collision'
    _TYPE = 'sdf'

    def __init__(self, default='__default__'):
        XMLString.__init__(self, default)


class Contact(XMLBase):
    _NAME = 'contact'
    _TYPE = 'sdf'

    _CHILDREN_CREATORS = dict(
        stiffness=dict(
            creator=Stiffness, default=[1e8], mode='simbody'),
        dissipation=dict(
            creator=Dissipation, default=[100], mode='simbody'),
        plastic_coef_restitution=dict(
            creator=PlasticCoefRestitution, default=[0.5], mode='simbody'),
        plastic_impact_velocity=dict(
            creator=PlasticImpactVelocity, default=[0.5], mode='simbody'),
        static_friction=dict(
            creator=StaticFriction, default=[0.9], mode='simbody'),
        dynamic_friction=dict(
            creator=DynamicFriction, default=[0.9], mode='simbody'),
        viscous_friction=dict(
            creator=ViscousFriction, default=[0], mode='simbody'),
        override_impact_capture_velocity=dict(
            creator=OverrideImpactCaptureVelocity,
            default=[0.001],
            mode='simbody'),
        override_stiction_transition_velocity=dict(
            creator=OverrideStictionTransitionVelocity,
            default=[0.001],
            mode='simbody'),
        ode=dict(
            creator=ODE,
            default=['contact'],
            optional=True,
            mode='collision'),
        bullet=dict(
            creator=Bullet,
            default=['contact'],
            optional=True,
            mode='collision'),
        collision=dict(
            creator=ContactCollisionName, mode='sensor'),
        topic=dict(
            creator=Topic, default=['__default_topic__'], mode='sensor'),
        collide_bitmask=dict(
            creator=CollideBitmask,
            default=[65535],
            optional=True,
            mode='collision'),
        collide_without_contact=dict(
            creator=CollideWithoutContact,
            default=[False],
            optional=True,
            mode='collision'),
        collide_without_contact_bitmask=dict(
            creator=CollideWithoutContactBitmask,
            default=[True],
            optional=True,
            mode='collision'),
        category_bitmask=dict(
            creator=CategoryBitmask,
            default=[65535],
            optional=True,
            mode='collision'),
        poissons_ratio=dict(
            creator=PoissonsRatio,
            default=[0.3],
            optional=True,
            mode='collision'),
        elastic_modulus=dict(
            creator=ElasticModulus,
            default=[-1],
            optional=True,
            mode='collision')
    )

    _MODES = ['simbody', 'collision', 'sensor']

    def __init__(self, mode='simbody'):
        XMLBase.__init__(self)
        self.reset(mode=mode)

    @property
    def stiffness(self):
        return self._get_child_element('stiffness')

    @stiffness.setter
    def stiffness(self, value):
        if self._mode != 'simbody':
            self.reset(mode='simbody')
        self._add_child_element('stiffness', value)

    @property
    def dissipation(self):
        return self._get_child_element('dissipation')

    @dissipation.setter
    def dissipation(self, value):
        if self._mode != 'simbody':
            self.reset(mode='simbody')
        self._add_child_element('dissipation', value)

    @property
    def plastic_coef_restitution(self):
        return self._get_child_element('plastic_coef_restitution')

    @plastic_coef_restitution.setter
    def plastic_coef_restitution(self, value):
        if self._mode != 'simbody':
            self.reset(mode='simbody')
        self._add_child_element('plastic_coef_restitution', value)

    @property
    def plastic_impact_velocity(self):
        return self._get_child_element('plastic_impact_velocity')

    @plastic_impact_velocity.setter
    def plastic_impact_velocity(self, value):
        if self._mode != 'simbody':
            self.reset(mode='simbody')
        self._add_child_element('plastic_impact_velocity', value)

    @property
    def static_friction(self):
        return self._get_child_element('static_friction')

    @static_friction.setter
    def static_friction(self, value):
        if self._mode != 'simbody':
            self.reset(mode='simbody')
        self._add_child_element('static_friction', value)

    @property
    def dynamic_friction(self):
        return self._get_child_element('dynamic_friction')

    @dynamic_friction.setter
    def dynamic_friction(self, value):
        if self._mode != 'simbody':
            self.reset(mode='simbody')
        self._add_child_element('dynamic_friction', value)

    @property
    def viscous_friction(self):
        return self._get_child_element('viscous_friction')

    @viscous_friction.setter
    def viscous_friction(self, value):
        if self._mode != 'simbody':
            self.reset(mode='simbody')
        self._add_child_element('viscous_friction', value)

    @property
    def override_impact_capture_velocity(self):
        return self._get_child_element('override_impact_capture_velocity')

    @override_impact_capture_velocity.setter
    def override_impact_capture_velocity(self, value):
        if self._mode != 'simbody':
            self.reset(mode='simbody')
        self._add_child_element('override_impact_capture_velocity', value)

    @property
    def override_stiction_transition_velocity(self):
        return self._get_child_element('override_stiction_transition_velocity')

    @override_stiction_transition_velocity.setter
    def override_stiction_transition_velocity(self, value):
        if self._mode != 'simbody':
            self.reset(mode='simbody')
        self._add_child_element('override_stiction_transition_velocity', value)

    @property
    def ode(self):
        return self._get_child_element('ode')

    @ode.setter
    def ode(self, value):
        if self._mode != 'collision':
            self.reset(mode='collision')
        self._add_child_element('ode', value)

    @property
    def bullet(self):
        return self._get_child_element('bullet')

    @bullet.setter
    def bullet(self, value):
        if self._mode != 'collision':
            self.reset(mode='collision')
        self._add_child_element('bullet', value)

    @property
    def collision(self):
        return self._get_child_element('collision')

    @collision.setter
    def collision(self, value):
        if self._mode != 'sensor':
            self.reset(mode='sensor')
        self._add_child_element('collision', value)

    @property
    def topic(self):
        return self._get_child_element('topic')

    @topic.setter
    def topic(self, value):
        if self._mode != 'sensor':
            self.reset(mode='sensor')
        self._add_child_element('topic', value)

    @property
    def collide_bitmask(self):
        return self._get_child_element('collide_bitmask')

    @collide_bitmask.setter
    def collide_bitmask(self, value):
        if self._mode != 'collision':
            self.reset(mode='collision')
        self._add_child_element('collide_bitmask', value)

    @property
    def collide_without_contact(self):
        return self._get_child_element('collide_without_contact')

    @collide_without_contact.setter
    def collide_without_contact(self, value):
        if self._mode != 'collision':
            self.reset(mode='collision')
        self._add_child_element('collide_without_contact', value)

    @property
    def collide_without_contact_bitmask(self):
        return self._get_child_element('collide_without_contact_bitmask')

    @collide_without_contact_bitmask.setter
    def collide_without_contact_bitmask(self, value):
        if self._mode != 'collision':
            self.reset(mode='collision')
        self._add_child_element('collide_without_contact_bitmask', value)

    @property
    def category_bitmask(self):
        return self._get_child_element('category_bitmask')

    @category_bitmask.setter
    def category_bitmask(self, value):
        if self._mode != 'collision':
            self.reset(mode='collision')
        self._add_child_element('category_bitmask', value)

    @property
    def poissons_ratio(self):
        return self._get_child_element('poissons_ratio')

    @poissons_ratio.setter
    def poissons_ratio(self, value):
        if self._mode != 'collision':
            self.reset(mode='collision')
        self._add_child_element('poissons_ratio', value)

    @property
    def elastic_modulus(self):
        return self._get_child_element('elastic_modulus')

    @elastic_modulus.setter
    def elastic_modulus(self, value):
        if self._mode != 'collision':
            self.reset(mode='collision')
        self._add_child_element('elastic_modulus', value)
