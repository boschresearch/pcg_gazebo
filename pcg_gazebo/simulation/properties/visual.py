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
from .material import Material
from .geometry import Geometry
from .pose import Pose
import collections


class Visual(object):
    def __init__(self,
                 name='visual',
                 pose=[0, 0, 0, 0, 0, 0],
                 cast_shadows=True,
                 transparency=0,
                 geometry_type=None,
                 geometry_args=None):
        self._sdf_visual = create_sdf_element('visual')
        self._sdf_visual.reset(with_optional_elements=True)
        self._sdf_visual.name = name
        self._include_in_sdf = dict(
            cast_shadows=True,
            pose=True,
            transparency=True,
            material=False
        )

        self._material = Material()
        self._geometry = Geometry(link_element='visual')
        self._mesh = None
        self._pose = Pose()

        mat = self._material.get_random_xkcd_material_as_sdf()
        self._default_display_color = mat.diffuse.value

        self.pose = pose
        self.transparency = transparency
        self.cast_shadows = cast_shadows

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

    def __str__(self):
        return self.to_sdf().to_xml_as_str(pretty_print=True)

    @property
    def sdf(self):
        return self._sdf_visual

    @property
    def name(self):
        return self._sdf_visual.name

    @name.setter
    def name(self, value):
        self._sdf_visual.name = value

    @property
    def cast_shadows(self):
        return self._sdf_visual.cast_shadows.value

    @cast_shadows.setter
    def cast_shadows(self, flag):
        self._sdf_visual.cast_shadows = flag

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
    def transparency(self):
        return self._sdf_visual.transparency.value

    @transparency.setter
    def transparency(self, value):
        assert 0 <= value <= 1, 'Transparency value must be in interval [0, 1]'
        self._sdf_visual.transparency = value

    @property
    def material(self):
        return self._sdf_visual.material

    @property
    def geometry(self):
        return self._geometry

    @property
    def is_mesh(self):
        return self._geometry.is_mesh

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
            # Transform center position wrt visual's pose
            center = self._pose.quat.rotate(center)
            center[0] += self.pose.x
            center[1] += self.pose.y
            center[2] += self.pose.z
        return center

    def set_material_script(
            self,
            name,
            uri='file://media/materials/scripts/gazebo.material'):
        self._sdf_visual.material = \
            self._material.get_script_material_as_sdf(name, uri)
        self.enable_property('material')

    def set_xkcd_color(self, name=None):
        if name is not None:
            if name in self._material.xkcd_colors:
                self._sdf_visual.material = \
                    self._material.get_xkcd_material_as_sdf(name)
        else:
            self._sdf_visual.material = \
                self._material.get_random_xkcd_material_as_sdf()
        self.enable_property('material')

    def set_color(self, r=None, g=None, b=None, a=1):
        self._sdf_visual.material = self._material.get_color_material(
            r, g, b, a)
        self.enable_property('material')

    def set_geometry(self, name, params):
        assert name in Geometry._GEO_TYPES, \
            'Invalid geometry type'

        self._geometry = Geometry(name, **params)

    def set_box_as_geometry(self, size=[1, 1, 1]):
        self._geometry.set_box(size)

    def set_sphere_as_geometry(self, radius):
        self._geometry.set_sphere(radius)

    def set_cylinder_as_geometry(self, length, radius):
        self._geometry.set_cylinder(radius=radius, length=length)

    def set_mesh_as_geometry(self, mesh, scale=[1, 1, 1], load_mesh=True):
        self._geometry.set_mesh(mesh=mesh, scale=scale, load_mesh=load_mesh)

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

    def to_sdf(self, resource_prefix='', model_folder=None,
               copy_resources=False):
        visual = create_sdf_element('visual')
        visual.name = self.name
        if len(resource_prefix) == 0:
            filename = self.name
        else:
            filename = '{}_{}'.format(
                resource_prefix,
                self.name)
        visual.geometry = self._geometry.to_sdf(
            filename=filename,
            model_folder=model_folder,
            copy_resources=copy_resources)
        if self.using_property('material'):
            visual.material = self._sdf_visual.material
        if self.using_property('pose'):
            visual.pose = self._pose.to_sdf()
        if self.using_property('cast_shadows'):
            visual.cast_shadows = self._sdf_visual.cast_shadows
        if self.using_property('transparency'):
            visual.transparency = self._sdf_visual.transparency
        return visual

    @staticmethod
    def from_sdf(sdf):
        assert sdf._NAME == 'visual', 'Input has to be a visual SDF element'
        visual = Visual()
        visual.name = sdf.name
        visual._geometry = Geometry.from_sdf(sdf.geometry)

        if sdf.cast_shadows is not None:
            visual.cast_shadows = sdf.cast_shadows.value
            visual.enable_property('cast_shadows')

        if sdf.transparency is not None:
            visual.transparency = sdf.transparency.value
            visual.enable_property('transparency')

        if sdf.pose is not None:
            visual.pose = Pose.from_sdf(sdf.pose)
            visual.enable_property('pose')

        if sdf.material is not None:
            visual._sdf_visual.material = sdf.material
            visual.enable_property('material')

        return visual

    def to_marker(self):
        if self._geometry is None:
            return None
        marker = self._geometry.to_marker()
        if marker.mesh_use_embedded_materials:
            marker.color.r = self._default_display_color[0]
            marker.color.g = self._default_display_color[1]
            marker.color.b = self._default_display_color[2]
            marker.color.a = self._default_display_color[3]
            marker.mesh_use_embedded_materials = False
        elif self._sdf_visual.material:
            if self._sdf_visual.material.diffuse:
                color = self._sdf_visual.material.diffuse.value
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 1
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0
        return marker
