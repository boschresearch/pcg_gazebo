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
import itertools
import numpy as np
from .mesh import Mesh
from .footprint import Footprint
from ...log import PCG_ROOT_LOGGER
try:
    from visualization_msgs.msg import Marker
    CONVERT_TO_MARKER = True
except ImportError:
    CONVERT_TO_MARKER = False


class Geometry(object):
    _GEO_TYPES = ['box', 'cylinder', 'sphere', 'plane', 'mesh']

    def __init__(self, geo_type=None, **kwargs):
        self._sdf = None
        self._mesh = None
        self._mesh_pkg_name = None
        self._mesh_resource = None
        self._geo_type = geo_type
        if geo_type is not None:
            assert geo_type in self._GEO_TYPES, \
                'Invalid geometry type, options={}'.format(self._GEO_TYPES)
            if geo_type == 'mesh':
                self.set_mesh(**kwargs)
            else:
                self._sdf = create_sdf_element(geo_type)
                if len(kwargs):
                    for tag in kwargs:
                        self.set_param(tag, kwargs[tag])

                if geo_type == 'box':
                    self._mesh = Mesh.create_box(
                        size=self.get_param('size'))
                elif geo_type == 'cylinder':
                    self._mesh = Mesh.create_cylinder(
                        radius=self.get_param('radius'),
                        height=self.get_param('length'))
                elif geo_type == 'sphere':
                    self._mesh = Mesh.create_sphere(
                        radius=self.get_param('radius'))
                elif geo_type == 'plane':
                    self._mesh = Mesh.create_box(
                        size=self.get_param('size') + [0.001])

    def get_samples(self):
        pnts = None
        if self.get_type() == 'sphere':
            n = 10
            radius = self.get_param('radius')
            for theta in np.linspace(0, 2 * np.pi, n / 2):
                for psi in np.linspace(0, 2 * np.pi, n / 2):
                    pnt = np.array([
                        radius * np.sin(theta) * np.cos(psi),
                        radius * np.sin(theta) * np.sin(psi),
                        radius * np.cos(theta)])
                    if pnts is None:
                        pnts = pnt
                    else:
                        pnts = np.vstack((pnts, pnt))
        elif self.get_type() == 'plane':
            size = self.get_param('size')
            for i in [-size[0] / 2, size[0] / 2]:
                for j in [-size[1] / 2, size[1] / 2]:
                    if pnts is None:
                        pnts = np.array([i, j])
                    else:
                        pnts = np.vstack((pnts, np.array([i, j])))
        elif self.get_type() == 'cylinder':
            n = 20

            radius = self.get_param('radius')
            length = self.get_param('length')

            for z in [-1 * length / 2, length / 2]:
                for a in np.linspace(0, 2 * np.pi, int(n / 2)):
                    pnt = np.array([radius * np.cos(a), radius * np.sin(a), z])
                    if pnts is None:
                        pnts = pnt
                    else:
                        pnts = np.vstack((pnts, pnt))
        elif self.get_type() == 'box':
            bounds = self.get_bounds()
            x = [bounds['lower_x'], bounds['upper_x']]
            y = [bounds['lower_y'], bounds['upper_y']]
            z = [bounds['lower_z'], bounds['upper_z']]

            for pnt in itertools.product(x, y, z):
                pnt = np.array(list(pnt))
                if pnts is None:
                    pnts = pnt
                else:
                    pnts = np.vstack((pnts, pnt))
        elif self.get_type() == 'mesh':
            pnts = self._mesh.get_samples(1000)

        return pnts

    def get_mesh(self, position=None, quat=None):
        from ...transformations import quaternion_matrix
        assert self._mesh is not None, 'No mesh found for this geometry'

        if position is None:
            position = np.array([0, 0, 0])

        if quat is None:
            quat = np.array([0, 0, 0, 1])

        transformed_mesh = self._mesh.apply_transform(
            position, quaternion_matrix(quat))

        return transformed_mesh

    def get_footprint(self, position=None, quat=None, use_bounding_box=False,
                      z_limits=None):
        from ...transformations import quaternion_matrix
        if position is None:
            position = np.array([0, 0, 0])

        if quat is None:
            quat = np.array([0, 0, 0, 1])

        footprint = None
        fp_poly = self._mesh.get_footprint_polygon(
            z_limits=z_limits,
            use_global_frame=True,
            transform=quaternion_matrix(quat),
            offset=position,
            use_bounding_box=use_bounding_box)

        if fp_poly is None:
            return None
        footprint = Footprint()
        footprint.add_polygon(fp_poly)

        return footprint

    def get_type(self):
        if self._sdf is not None:
            return self._sdf._NAME
        elif self._mesh is not None:
            return 'mesh'
        else:
            return None

    def get_param(self, param_name):
        if self._sdf is None and self._mesh is None:
            return None
        if not hasattr(self._sdf, param_name):
            return None
        if self._sdf is not None:
            return getattr(self._sdf, param_name).value
        elif self._mesh is not None:
            return getattr(self._mesh, param_name)

    def set_param(self, param_name, value):
        if self._sdf is None and self._mesh is None:
            return False
        if not hasattr(self._sdf, param_name):
            return False
        if self._sdf is not None:
            setattr(self._sdf, param_name, value)
        elif self._mesh is not None:
            setattr(self._mesh, param_name, value)
        return True

    def get_bounds(self):
        if self._sdf is not None:
            if self._sdf._NAME == 'box':
                size = self._sdf.size.value
                bounds = dict(
                    lower_x=-1 * size[0] / 2.0,
                    upper_x=size[0] / 2.0,
                    lower_y=-1 * size[1] / 2.0,
                    upper_y=size[1] / 2.0,
                    lower_z=-1 * size[2] / 2.0,
                    upper_z=size[2] / 2.0
                )
            elif self._sdf._NAME == 'cylinder':
                radius = self._sdf.radius.value
                length = self._sdf.length.value
                bounds = dict(
                    lower_x=-1 - radius / 2.0,
                    upper_x=radius / 2.0,
                    lower_y=-1 * radius / 2.0,
                    upper_y=radius / 2.0,
                    lower_z=-1 * length / 2.0,
                    upper_z=length / 2.0
                )
            elif self._sdf._NAME == 'sphere':
                radius = self._sdf.radius.value
                bounds = dict(
                    lower_x=-1 - radius / 2.0,
                    upper_x=radius / 2.0,
                    lower_y=-1 * radius / 2.0,
                    upper_y=radius / 2.0,
                    lower_z=-1 * radius / 2.0,
                    upper_z=radius / 2.0
                )
            elif self._sdf._NAME == 'plane':
                size = self._sdf.size.value
                bounds = dict(
                    lower_x=-size[0] / 2,
                    upper_x=size[0] / 2,
                    lower_y=-size[1] / 2,
                    upper_y=size[1] / 2,
                    lower_z=0,
                    upper_z=0
                )
        elif self._mesh is not None:
            if self._mesh.mesh is None:
                self._mesh.load_mesh()
                self._mesh.compute_bounds()
            bounds = self._mesh.bounds
        else:
            return None

        return bounds

    def get_center(self):
        if self._sdf is not None:
            return [0, 0, 0]
        elif self._mesh is not None:
            return self._mesh.center
        else:
            return None

    def set_box(self, size):
        assert isinstance(size, collections.Iterable), \
            'Invalid input array'
        size = list(size)
        assert len(size) == 3, 'Input size vector must have 3 elements'
        for elem in size:
            assert elem > 0, 'Size element must be greater than zero'
        self._sdf = create_sdf_element('box')
        self.set_param('size', size)
        self._mesh = Mesh.create_box(
            size=size)
        self._geo_type = 'box'

    def set_cylinder(self, radius, length):
        assert radius > 0, 'Radius must be greater than zero'
        assert length > 0, 'Length must be greater than zero'
        self._sdf = create_sdf_element('cylinder')
        self.set_param('radius', radius)
        self.set_param('length', length)
        self._mesh = Mesh.create_cylinder(
            radius=radius,
            height=length)
        self._geo_type = 'cylinder'

    def set_sphere(self, radius):
        assert radius > 0, 'Radius must be greater than zero'
        self._sdf = create_sdf_element('sphere')
        self.set_param('radius', radius)
        self._mesh = Mesh.create_sphere(
            radius=radius)
        self._geo_type = 'sphere'

    def set_plane(self, size, normal):
        assert isinstance(size, collections.Iterable), \
            'Invalid size array'
        size = list(size)
        assert len(size) == 2, 'Input size vector must have 2 elements'
        for elem in size:
            assert elem > 0, 'Size element must be greater than zero'

        assert isinstance(normal, collections.Iterable), \
            'Invalid normal array'
        normal = list(normal)
        assert len(normal) == 3, 'Input normal vector must have 3 elements'
        for elem in normal:
            assert elem >= 0, \
                'Normal element must be greater than or equal to zero'
        self._sdf = create_sdf_element('plane')
        self.set_param('size', size)
        self.set_param('normal', normal)
        self._mesh = Mesh.create_box(
            size=size + [0.001])
        self._geo_type = 'plane'

    def set_mesh(self, mesh, scale=[1, 1, 1], load_mesh=True):
        if isinstance(mesh, str):
            self._mesh = Mesh(mesh, load_mesh)
        else:
            self._mesh = Mesh.from_mesh(mesh, scale)
        # self._sdf = self._mesh.to_sdf()
        self._mesh.scale = scale
        self._geo_type = 'mesh'

    def to_sdf(
            self,
            mesh_filename=None,
            model_folder=None,
            copy_resources=False):
        PCG_ROOT_LOGGER.info('Convert geometry to SDF')
        sdf = create_sdf_element('geometry')

        if self._sdf is not None or self._geo_type == 'mesh':
            if self._geo_type == 'mesh':
                if self._mesh.filename is not None:
                    mesh_filename = None
                self._sdf = self._mesh.to_sdf(
                    mesh_filename=mesh_filename,
                    model_folder=model_folder,
                    copy_resources=copy_resources)
            setattr(sdf, self._sdf.xml_element_name, self._sdf)
        else:
            sdf = None
        return sdf

    @staticmethod
    def from_sdf(sdf):
        assert sdf._NAME == 'geometry', 'Only geometries can be parsed'
        geo = Geometry()
        if sdf.box is not None:
            geo.set_box(sdf.box.size.value)
        elif sdf.cylinder is not None:
            geo.set_cylinder(
                radius=sdf.cylinder.radius.value,
                length=sdf.cylinder.length.value)
        elif sdf.sphere is not None:
            geo.set_sphere(sdf.sphere.radius.value)
        elif sdf.plane is not None:
            geo.set_plane(
                size=sdf.plane.size.value,
                normal=sdf.plane.normal.value)
        elif sdf.mesh is not None:
            geo.set_mesh(
                mesh=sdf.mesh.uri.value,
                scale=sdf.mesh.scale.value,
                load_mesh=False)
        return geo

    def to_marker(self):
        if not CONVERT_TO_MARKER:
            return None
        if self._sdf is None and self._mesh is None:
            return None

        marker = Marker()
        if self._sdf:
            if self._sdf._NAME == 'sphere':
                marker.type = Marker.SPHERE
                diameter = 2 * self.get_param('radius')
                marker.scale.x = diameter
                marker.scale.y = diameter
                marker.scale.z = diameter
            elif self._sdf._NAME == 'box':
                marker.type = Marker.CUBE
                size = self.get_param('size')
                marker.scale.x = size[0]
                marker.scale.y = size[1]
                marker.scale.z = size[2]
            elif self._sdf._NAME == 'cylinder':
                marker.type = Marker.CYLINDER
                diameter = 2 * self.get_param('radius')
                marker.scale.x = diameter
                marker.scale.y = diameter
                marker.scale.z = self.get_param('length')
            else:
                return None
        elif self._mesh:
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = 'file://' + self._mesh.filename
            marker.scale.x = self._mesh.scale[0]
            marker.scale.y = self._mesh.scale[1]
            marker.scale.z = self._mesh.scale[2]
            marker.mesh_use_embedded_materials = True

        return marker
