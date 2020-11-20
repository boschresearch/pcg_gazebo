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

from ...parsers.sdf import create_sdf_element
import collections
from .geometry import Geometry
from .pose import Pose
from ...log import PCG_ROOT_LOGGER


class Collision(object):
    def __init__(self,
                 name='collision',
                 pose=[0, 0, 0, 0, 0, 0],
                 geometry_type=None,
                 geometry_args=None,
                 mu=None,
                 mu2=None,
                 friction=None,
                 friction2=None,
                 slip1=None,
                 slip2=None,
                 rolling_friction=None,
                 fdir1=None,
                 max_contacts=None,
                 soft_cfm=None,
                 soft_erp=None,
                 kp=None,
                 kd=None,
                 max_vel=None,
                 min_depth=None,
                 split_impulse=None,
                 split_impulse_penetration_threshold=None,
                 restitution_coefficient=None,
                 threshold=None,
                 collide_bitmask=None,
                 collide_without_contact=None,
                 collide_without_contact_bitmask=None,
                 category_bitmask=None,
                 poissons_ratio=None,
                 elastic_modulus=None):

        self._sdf_collision = create_sdf_element(
            'collision', 'link')
        self._include_in_sdf = dict(
            max_contacts=True,
            pose=True,
            friction=False,
            bounce=False,
            contact=False
        )
        self._geometry = Geometry(link_element='collision')
        self._pose = Pose()

        # Setting the input parameters
        self.name = name
        self.pose = pose

        if geometry_type is not None and geometry_args is not None:
            if geometry_type == 'cylinder':
                self.set_cylinder_as_geometry(**geometry_args)
            elif geometry_type == 'sphere':
                self.set_sphere_as_geometry(**geometry_args)
            elif geometry_type == 'mesh':
                self.set_mesh_as_geometry(**geometry_args)
            elif geometry_type == 'box':
                self.set_box_as_geometry(**geometry_args)
            elif geometry_type == 'plane':
                self.set_plane_as_geometry(**geometry_args)

        if max_contacts is not None:
            self.max_contacts = max_contacts

        self.set_ode_friction_params(
            mu=mu,
            mu2=mu2,
            slip1=slip1,
            slip2=slip2,
            fdir1=fdir1
        )
        self.set_bullet_friction_params(
            friction=friction,
            friction2=friction2,
            fdir1=fdir1,
            rolling_friction=rolling_friction
        )

        self.set_ode_contact_params(
            soft_cfm=soft_cfm,
            soft_erp=soft_erp,
            kp=kp,
            kd=kd,
            max_vel=max_vel,
            min_depth=min_depth
        )

        self.set_bullet_contact_params(
            soft_cfm, soft_erp, kp,
            kd, split_impulse,
            split_impulse_penetration_threshold)

        self.set_bounce_params(
            restitution_coefficient=restitution_coefficient,
            threshold=threshold)

        self.set_contact_params(
            collide_bitmask=collide_bitmask,
            collide_without_contact=collide_without_contact,
            collide_without_contact_bitmask=collide_without_contact_bitmask,
            category_bitmask=category_bitmask,
            poissons_ratio=poissons_ratio,
            elastic_modulus=elastic_modulus)

    def __str__(self):
        return self.to_sdf().to_xml_as_str(pretty_print=True)

    @property
    def sdf(self):
        return self._sdf_collision

    @property
    def name(self):
        return self._sdf_collision.name

    @name.setter
    def name(self, value):
        self._sdf_collision.name = value

    @property
    def max_contacts(self):
        return self._sdf_collision.max_contacts.value

    @property
    def is_mesh(self):
        return self._geometry.is_mesh

    @max_contacts.setter
    def max_contacts(self, max_contacts):
        self._sdf_collision.max_contacts = max_contacts
        PCG_ROOT_LOGGER.info(
            'Set max. contacts, collision={}, max_contacts={}'.format(
                self.name, max_contacts))

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, vec):
        if isinstance(vec, Pose):
            self._pose = vec
        else:
            assert isinstance(vec, collections.Iterable), \
                'Input vector must be iterable'
            assert len(vec) == 6 or len(vec) == 7, \
                'Input vector must have either 6 or 7 elements'
            for item in vec:
                assert isinstance(item, float) or isinstance(item, int), \
                    'Each pose element must be either a float or an integer'

            self._pose = Pose(pos=vec[0:3], rot=vec[3::])

    @property
    def geometry(self):
        return self._geometry

    def get_bounds(self):
        bounds = self._geometry.get_bounds()
        if bounds is not None:
            # Apply collision element transformations
            lower = [bounds['lower_x'], bounds['lower_y'], bounds['lower_z']]
            upper = [bounds['upper_x'], bounds['upper_y'], bounds['upper_z']]

            lower = self._pose.quat.rotate(lower)
            upper = self._pose.quat.rotate(upper)

            bounds['lower_x'] = lower[0] + self.pose.x
            bounds['upper_x'] = upper[0] + self.pose.x
            bounds['lower_y'] = lower[1] + self.pose.y
            bounds['upper_y'] = upper[1] + self.pose.y
            bounds['lower_z'] = lower[2] + self.pose.z
            bounds['upper_z'] = upper[2] + self.pose.z
        return bounds

    def get_center(self):
        center = self._geometry.get_center()
        if center is not None:
            # Transform center position wrt collision's pose
            center = self._pose.quat.rotate(center)
            center[0] += self.pose.x
            center[1] += self.pose.y
            center[2] += self.pose.z
        return center

    def set_geometry(self, name, params):
        assert name in Geometry._GEO_TYPES, \
            'Invalid geometry type, options={}'.format(Geometry._GEO_TYPES)

        self._geometry = Geometry(name, **params)

    def set_box_as_geometry(self, size=[1, 1, 1]):
        self._geometry.set_box(size)

    def set_sphere_as_geometry(self, radius):
        self._geometry.set_sphere(radius)

    def set_cylinder_as_geometry(self, length, radius):
        self._geometry.set_cylinder(radius=radius, length=length)

    def set_mesh_as_geometry(self, mesh, scale=[1, 1, 1], load_mesh=True):
        self._geometry.set_mesh(mesh, scale=scale, load_mesh=load_mesh)

    def set_plane_as_geometry(self, size, normal):
        self._geometry.set_plane(size=size, normal=normal)

    def enable_property(self, name):
        assert name in self._include_in_sdf, 'Invalid property name'
        self._include_in_sdf[name] = True

    def disable_property(self, name):
        assert name in self._include_in_sdf, 'Invalid property name'
        self._include_in_sdf[name] = False

    def using_property(self, name):
        assert name in self._include_in_sdf, 'Invalid property name'
        return self._include_in_sdf[name]

    def set_contact_params(
            self,
            collide_bitmask,
            collide_without_contact,
            collide_without_contact_bitmask,
            category_bitmask,
            poissons_ratio,
            elastic_modulus,
            ode_parameters=None,
            bullet_parameters=None):
        try:
            if any(
                x is not None for x in [
                    collide_bitmask,
                    collide_without_contact,
                    collide_without_contact_bitmask,
                    category_bitmask,
                    poissons_ratio,
                    elastic_modulus]):
                if self.sdf.surface is None:
                    self.sdf.surface = create_sdf_element('surface')
                if self.sdf.surface.contact is None:
                    self.sdf.surface.contact = create_sdf_element('contact')

            if collide_bitmask is not None:
                self.sdf.surface.contact.collide_bitmask = collide_bitmask
            if collide_without_contact is not None:
                self.sdf.surface.contact.collide_without_contact = \
                    collide_without_contact
            if collide_without_contact_bitmask is not None:
                self.sdf.surface.contact.collide_without_contact_bitmask = \
                    collide_without_contact_bitmask
            if category_bitmask is not None:
                self.sdf.surface.contact.category_bitmask = category_bitmask
            if poissons_ratio is not None:
                self.sdf.surface.contact.poissons_ratio = poissons_ratio
            if elastic_modulus is not None:
                self.sdf.surface.contact.elastic_modulus = elastic_modulus
            if ode_parameters is not None and isinstance(ode_parameters, dict):
                if not self.set_ode_contact_params(**ode_parameters):
                    return False
            if bullet_parameters is not None and isinstance(
                    bullet_parameters, dict):
                if not self.set_bullet_contact_params(**bullet_parameters):
                    return False
            if any(
                x is not None for x in [
                    collide_bitmask,
                    collide_without_contact,
                    collide_without_contact_bitmask,
                    category_bitmask,
                    poissons_ratio,
                    elastic_modulus,
                    ode_parameters,
                    bullet_parameters]):
                self.enable_property('contact')
            return True
        except AssertionError as ex:
            PCG_ROOT_LOGGER.error('Error setting bounce parameters, '
                                  'message={}'.format(ex))
            return False

    def get_bounce_param(self, tag):
        assert tag in ['restitution_coefficient', 'threshold'], \
            'Invalid bounce parameter name'
        try:
            param = getattr(self.sdf.surface.bounce, tag).value
        except BaseException:
            param = None
        return param

    def set_bounce_params(self, restitution_coefficient=None, threshold=None):
        try:
            if any(
                x is not None for x in [
                    restitution_coefficient,
                    threshold]):
                if self.sdf.surface is None:
                    self.sdf.surface = create_sdf_element('surface')
                if self.sdf.surface.bounce is None:
                    self.sdf.surface.bounce = create_sdf_element('bounce')

            if restitution_coefficient is not None:
                self.sdf.surface.bounce.restitution_coefficient = \
                    restitution_coefficient
            if threshold is not None:
                self.sdf.surface.bounce.threshold = threshold
            if any(
                x is not None for x in [
                    restitution_coefficient,
                    threshold]):
                self.enable_property('bounce')
                PCG_ROOT_LOGGER.info('Set bounce parameters, SDF={}'.format(
                    self.sdf.surface.bounce))
            return True
        except AssertionError as ex:
            PCG_ROOT_LOGGER.error('Error setting bounce parameters, '
                                  'message={}'.format(ex))
            return False

    def set_ode_friction_params(
            self,
            mu=None,
            mu2=None,
            slip1=None,
            slip2=None,
            fdir1=None):
        try:
            if any(x is not None for x in [mu, mu2, slip1, slip2, fdir1]):
                if self.sdf.surface is None:
                    self.sdf.surface = create_sdf_element('surface')
                if self.sdf.surface.friction is None:
                    self.sdf.surface.friction = create_sdf_element(
                        'friction', 'surface')
                if self.sdf.surface.friction.ode is None:
                    ode = create_sdf_element('ode', 'collision')
                    self.sdf.surface.friction.ode = ode
            if mu is not None:
                self.sdf.surface.friction.ode.mu = mu
            if mu2 is not None:
                self.sdf.surface.friction.ode.mu2 = mu2
            if slip1 is not None:
                self.sdf.surface.friction.ode.slip1 = slip1
            if slip2 is not None:
                self.sdf.surface.friction.ode.slip2 = slip2
            if fdir1 is not None:
                self.sdf.surface.friction.ode.fdir1 = fdir1
            if any(x is not None for x in [mu, mu2, slip1, slip2, fdir1]):
                self.enable_property('friction')
                PCG_ROOT_LOGGER.info(
                    'Set ODE friction parameters, SDF={}'.format(
                        self.sdf.surface.friction.ode))
            return True
        except AssertionError as ex:
            PCG_ROOT_LOGGER.error('Error setting ODE friction parameters, '
                                  'message={}'.format(ex))
            return False

    def get_ode_friction_param(self, tag):
        assert tag in ['mu', 'mu2', 'slip1', 'slip2', 'fdir1'], \
            'Invalid ODE friction parameter name'
        try:
            param = getattr(self.sdf.surface.friction.ode, tag).value
        except BaseException:
            param = None
        return param

    def set_ode_contact_params(
            self,
            soft_cfm=None,
            soft_erp=None,
            kp=None,
            kd=None,
            max_vel=None,
            min_depth=None):
        try:
            if any(x is not None for x in [soft_cfm, soft_erp, kp, kd, max_vel,
                                           min_depth]):
                if self.sdf.surface is None:
                    self.sdf.surface = create_sdf_element('surface')
                if self.sdf.surface.contact is None:
                    self.sdf.surface.contact = create_sdf_element('contact')
                if self.sdf.surface.contact.ode is None:
                    ode = create_sdf_element('ode', 'contact')
                    self.sdf.surface.contact.ode = ode
            if soft_cfm is not None:
                self.sdf.surface.contact.ode.soft_cfm = soft_cfm
            if soft_erp is not None:
                self.sdf.surface.contact.ode.soft_erp = soft_erp
            if kp is not None:
                self.sdf.surface.contact.ode.kp = kp
            if kd is not None:
                self.sdf.surface.contact.ode.kd = kd
            if max_vel is not None:
                self.sdf.surface.contact.ode.max_vel = max_vel
            if min_depth is not None:
                self.sdf.surface.contact.ode.min_depth = min_depth
            if any(x is not None for x in [soft_cfm, soft_erp, kp, kd, max_vel,
                                           min_depth]):
                self.enable_property('contact')
                PCG_ROOT_LOGGER.info(
                    'Set ODE contact parameters, SDF={}'.format(
                        self.sdf.surface.contact.ode))
            return True
        except AssertionError as ex:
            PCG_ROOT_LOGGER.error('Error setting ODE contact parameters, '
                                  'message={}'.format(ex))
            return False

    def get_ode_contact_param(self, tag):
        assert tag in ['soft_cfm', 'soft_erp', 'kp', 'kd', 'max_vel',
                       'min_depth'], 'Invalid ODE contact parameter name'
        try:
            param = getattr(self.sdf.surface.contact.ode, tag).value
        except BaseException:
            param = None
        return param

    def set_bullet_friction_params(self, friction=None, friction2=None,
                                   fdir1=None, rolling_friction=None):
        try:
            if any(
                x is not None for x in [
                    friction,
                    friction2,
                    fdir1,
                    rolling_friction]):
                if self.sdf.surface is None:
                    self.sdf.surface = create_sdf_element('surface')
                if self.sdf.surface.friction is None:
                    self.sdf.surface.friction = create_sdf_element(
                        'friction', 'surface')
                if self.sdf.surface.friction.bullet is None:
                    bullet = create_sdf_element('bullet', 'collision')
                    self.sdf.surface.friction.bullet = bullet

            if friction is not None:
                self.sdf.surface.friction.bullet.friction = friction
            if friction2 is not None:
                self.sdf.surface.friction.bullet.friction2 = friction2
            if fdir1 is not None:
                self.sdf.surface.friction.bullet.fdir1 = fdir1
            if rolling_friction is not None:
                self.sdf.surface.friction.bullet.rolling_friction = \
                    rolling_friction
            if any(
                x is not None for x in [
                    friction,
                    friction2,
                    fdir1,
                    rolling_friction]):
                self.enable_property('friction')
                PCG_ROOT_LOGGER.info(
                    'Set Bullet friction parameters, SDF={}'.format(
                        self.sdf.surface.friction.bullet))
            return True
        except AssertionError as ex:
            PCG_ROOT_LOGGER.error('Error setting Bullet friction parameters, '
                                  'message={}'.format(ex))
            return False

    def get_bullet_friction_param(self, tag):
        assert tag in ['friction', 'friction2', 'rolling_friction', 'fdir1'], \
            'Invalid Bullet friction parameter name'
        try:
            param = getattr(self.sdf.surface.friction.bullet, tag).value
        except BaseException:
            param = None
        return param

    def set_bullet_contact_params(self, soft_cfm=None, soft_erp=None, kp=None,
                                  kd=None, split_impulse=None,
                                  split_impulse_penetration_threshold=None):
        try:
            if any(
                x is not None for x in [
                    soft_cfm,
                    soft_erp,
                    kp,
                    kd,
                    split_impulse,
                    split_impulse_penetration_threshold]):
                if self.sdf.surface is None:
                    self.sdf.surface = create_sdf_element('surface')
                if self.sdf.surface.contact is None:
                    self.sdf.surface.contact = create_sdf_element('contact')
                if self.sdf.surface.contact.bullet is None:
                    bullet = create_sdf_element('bullet', 'contact')
                    self.sdf.surface.contact.bullet = bullet
            if soft_cfm is not None:
                self.sdf.surface.contact.bullet.soft_cfm = soft_cfm
            if soft_erp is not None:
                self.sdf.surface.contact.bullet.soft_erp = soft_erp
            if kp is not None:
                self.sdf.surface.contact.bullet.kp = kp
            if kd is not None:
                self.sdf.surface.contact.bullet.kd = kd
            if split_impulse is not None:
                self.sdf.surface.contact.bullet.split_impulse = split_impulse
            if split_impulse_penetration_threshold is not None:
                setattr(
                    self.sdf.surface.contact.bullet,
                    'split_impulse_penetration_threshold',
                    split_impulse_penetration_threshold)
            if any(
                x is not None for x in [
                    soft_cfm,
                    soft_erp,
                    kp,
                    kd,
                    split_impulse,
                    split_impulse_penetration_threshold]):
                self.enable_property('contact')
                PCG_ROOT_LOGGER.info(
                    'Set Bullet contact parameters, SDF={}'.format(
                        self.sdf.surface.contact.bullet))
            return True
        except AssertionError as ex:
            PCG_ROOT_LOGGER.error('Error setting Bullet contact parameters, '
                                  'message={}'.format(ex))
            return False

    def get_bullet_contact_param(self, tag):
        assert tag in ['soft_cfm', 'soft_erp', 'kp', 'kd', 'split_impulse',
                       'split_impulse_penetration_threshold'], \
            'Invalid ODE contact parameter name'
        try:
            param = getattr(self.sdf.surface.contact.bullet, tag).value
        except BaseException:
            param = None
        return param

    def set_physics(
            self,
            mu=None,
            mu2=None,
            friction=None,
            friction2=None,
            slip1=None,
            slip2=None,
            rolling_friction=None,
            fdir1=None,
            max_contacts=None,
            soft_cfm=None,
            soft_erp=None,
            kp=None,
            kd=None,
            max_vel=None,
            min_depth=None,
            split_impulse=None,
            split_impulse_penetration_threshold=None,
            restitution_coefficient=None,
            threshold=None,
            collide_bitmask=None,
            collide_without_contact=None,
            collide_without_contact_bitmask=None,
            category_bitmask=None,
            poissons_ratio=None,
            elastic_modulus=None):

        PCG_ROOT_LOGGER.info('Setting collision physics parameters')

        if max_contacts is not None:
            self.max_contacts = max_contacts

        self.set_ode_friction_params(
            mu=mu,
            mu2=mu2,
            slip1=slip1,
            slip2=slip2,
            fdir1=fdir1
        )
        self.set_bullet_friction_params(
            friction=friction,
            friction2=friction2,
            fdir1=fdir1,
            rolling_friction=rolling_friction
        )

        self.set_ode_contact_params(
            soft_cfm=soft_cfm,
            soft_erp=soft_erp,
            kp=kp,
            kd=kd,
            max_vel=max_vel,
            min_depth=min_depth
        )

        self.set_bullet_contact_params(
            soft_cfm, soft_erp, kp,
            kd, split_impulse,
            split_impulse_penetration_threshold)

        self.set_bounce_params(
            restitution_coefficient=restitution_coefficient,
            threshold=threshold)

        self.set_contact_params(
            collide_bitmask=collide_bitmask,
            collide_without_contact=collide_without_contact,
            collide_without_contact_bitmask=collide_without_contact_bitmask,
            category_bitmask=category_bitmask,
            poissons_ratio=poissons_ratio,
            elastic_modulus=elastic_modulus)

    def to_sdf(self, resource_prefix='', model_folder=None,
               copy_resources=False):
        collision = create_sdf_element('collision', 'link')
        collision.name = self.name
        if len(resource_prefix) == 0:
            filename = self.name
        else:
            filename = '{}_{}'.format(
                resource_prefix,
                self.name)
        collision.geometry = self._geometry.to_sdf(
            filename=filename,
            model_folder=model_folder,
            copy_resources=copy_resources)
        if self.using_property('pose'):
            collision.pose = self._pose.to_sdf()
        if self._sdf_collision.max_contacts is not None:
            collision.max_contacts = self._sdf_collision.max_contacts
        if self.sdf.surface is not None:
            if self.sdf.surface.friction is not None:
                if collision.surface is None:
                    collision.surface = create_sdf_element('surface')
                collision.surface.friction = self.sdf.surface.friction
            if self.sdf.surface.bounce is not None:
                if collision.surface is None:
                    collision.surface = create_sdf_element('surface')
                collision.surface.bounce = self.sdf.surface.bounce
            if self.sdf.surface.contact is not None:
                if collision.surface is None:
                    collision.surface = create_sdf_element('surface')
                collision.surface.contact = self.sdf.surface.contact
        return collision

    @staticmethod
    def from_sdf(sdf):
        assert sdf._NAME == 'collision', \
            'Only collision elements can be parsed'
        collision = Collision()
        collision.name = sdf.name
        collision.max_contacts = \
            20 if sdf.max_contacts is None else sdf.max_contacts.value

        if sdf.pose is not None:
            collision.pose = Pose.from_sdf(sdf.pose)

        if sdf.surface is not None:
            collision._sdf_collision.surface = sdf.surface
            if collision._sdf_collision.surface.friction is not None:
                collision.enable_property('friction')
            if collision._sdf_collision.surface.bounce is not None:
                collision.enable_property('bounce')
            if collision._sdf_collision.surface.contact is not None:
                collision.enable_property('contact')

        collision._geometry = Geometry.from_sdf(sdf.geometry)

        return collision
