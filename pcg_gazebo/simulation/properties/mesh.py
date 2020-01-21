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
import os
import numpy as np
import collections
import trimesh
import logging
from copy import deepcopy
from shapely.affinity import translate
from shapely.geometry import Polygon, MultiPolygon, LineString, \
    MultiPoint
from shapely.ops import unary_union, triangulate, polygonize
from ...log import PCG_ROOT_LOGGER
from ...parsers.sdf import create_sdf_element
from ...path import Path

# Disabling trimesh's logging messages
logger = logging.getLogger('trimesh')
logger.disabled = True


class Mesh(object):
    def __init__(self, filename=None, load_mesh=False):
        self._uri = None
        self._filename = None
        if filename is not None:
            assert isinstance(filename, str), 'Input filename is not a string'
            PCG_ROOT_LOGGER.info('Mesh created from file={}'.format(filename))
            self._uri = Path(filename)
            self._filename = self._uri.absolute_uri
        self._mesh = None
        self._bounds = None
        self._center = None
        self._scale = [1, 1, 1]

        self._footprint_states = list()

        if load_mesh:
            self.load_mesh()
            self.compute_bounds()

    @staticmethod
    def create_sphere(radius=1):
        assert radius > 0, 'Sphere radius must be greater than zero'
        mesh = Mesh()
        mesh._mesh = trimesh.creation.icosphere(radius=radius)
        PCG_ROOT_LOGGER.info(
            'Sphere mesh created, radius [m]={}'.format(radius))
        return mesh

    @staticmethod
    def create_cylinder(radius=1, height=1):
        assert radius > 0, 'Cylinder radius must be greater than zero'
        assert height > 0, 'Cylinder height must be greater than zero'
        mesh = Mesh()
        mesh._mesh = trimesh.creation.cylinder(radius=radius, height=height)
        PCG_ROOT_LOGGER.info(
            'Cylinder mesh created, radius [m]={}, height [m]={}'.format(
                radius, height))
        return mesh

    @staticmethod
    def create_capsule(radius=1, height=1):
        assert radius > 0, 'Capsule radius must be greater than zero'
        assert height > 0, 'Capsule height must be greater than zero'
        mesh = Mesh()
        mesh._mesh = trimesh.creation.capsule(radius=radius, height=height)
        PCG_ROOT_LOGGER.info(
            'Capsule mesh created, radius [m]={}, height [m]={}'.format(
                radius, height))
        return mesh

    @staticmethod
    def create_box(size=[1, 1, 1]):
        assert isinstance(size, collections.Iterable), 'Size is not an array'
        vec = list(size)
        assert len(vec) == 3, 'Input size array must have 3 elements'
        for elem in vec:
            assert elem > 0, 'Size vector components must be greater than zero'

        mesh = Mesh()
        mesh._mesh = trimesh.creation.box(extents=size)
        PCG_ROOT_LOGGER.info('Box mesh created, size={}'.format(size))
        return mesh

    @staticmethod
    def from_mesh(mesh, scale=[1, 1, 1]):
        assert isinstance(
            mesh, trimesh.Trimesh), 'Invalid mesh structure input'
        output = Mesh()
        output._mesh = mesh
        output._scale = scale
        PCG_ROOT_LOGGER.info('Mesh created from trimesh.Trimesh object')
        return output

    @property
    def filename(self):
        if self._uri is not None:
            return self._uri.absolute_uri
        else:
            return None

    @filename.setter
    def filename(self, value):
        assert isinstance(value, str), 'Input filename is not a string'
        self._uri = Path(value)
        self._filename = self._uri.absolute_uri

    @property
    def mesh(self):
        if self.load_mesh():
            return self._mesh
        return None

    @property
    def bounds(self):
        self.compute_bounds()
        return self._bounds

    @property
    def center(self):
        if self._mesh is None or self._bounds is None or self._center is None:
            self.compute_bounds()
        return self._center

    @property
    def scale(self):
        return self._scale

    @scale.setter
    def scale(self, vec):
        assert isinstance(vec, collections.Iterable), 'Input is not an array'
        vec = list(vec)
        assert len(vec) == 3, 'Input scale array must have 3 elements'
        for elem in vec:
            assert elem > 0, 'Scale vector' \
                ' components must be greater than zero'
        self._scale = vec

    @property
    def n_vertices(self):
        self.load_mesh()
        n_vertices = 0
        for mesh in self.get_meshes():
            n_vertices += mesh.vertices.shape[0]
        return n_vertices

    @property
    def vertices(self):
        self.load_mesh()
        vertices = None
        for mesh in self.get_meshes():
            if vertices is None:
                vertices = mesh.vertices
            else:
                vertices = np.vstack((vertices, mesh.vertices))
        return vertices

    @property
    def volume(self):
        self.load_mesh()
        volume = 0
        for mesh in self.get_meshes():
            volume += mesh.volume
        return volume

    @staticmethod
    def _multiline2points(lines):
        points = None
        for l in lines.geoms:
            line_points = np.array([[x, y] for x, y in zip(*l.xy)])
            if points is None:
                points = line_points
            else:
                points = np.vstack([points, line_points])

        points = np.unique(points, axis=0)
        return points

    def _store_footprint_state(self):
        pass

    def _get_footprint_state(self):
        pass

    def _get_combined_sections(self, mesh, origin, plane_normal, z_levels):
        # The sections are going to be taking using the plane_origin as
        # a reference, therefore, the z_levels must be relative to the
        # plane_origin
        # If the use_global_frame input is true, then the origin will
        # be set as the input origin vector
        sections = mesh.section_multiplane(
            plane_origin=origin,
            plane_normal=plane_normal,
            heights=z_levels)
        fp_offset = [mesh.bounds[0, 0], mesh.bounds[0, 1]]

        try:
            # Filter out sections that are invalid
            sections = [s for s in sections if s is not None]

            combined_section = np.sum(sections)
            # Retrieve all vertices, Path2D provides the index for the point
            vertices = combined_section.vertices
            # List of closed polygons
            closed_polys = list()

            for trimesh_line in combined_section.entities:
                if trimesh_line.closed:
                    points = vertices[trimesh_line.points]
                    # Create polygon
                    closed_polys.append(
                        translate(
                            Polygon(points),
                            fp_offset[0],
                            fp_offset[1]))

            # Combine all closed polygons into one
            closed_polys = unary_union(closed_polys)
            return closed_polys, combined_section
        except AttributeError:
            return None, None

    def _slice_mesh_with_plane(self, plane_normal=[0, 0, 1], plane_origin=None,
                               transform=None, offset=[0, 0, 0]):
        assert len(
            plane_normal) == 3, 'Plane normal' \
            ' vector must have three components'
        assert np.sum(
            plane_normal) == 1, 'Plane normal vector must be a unit vector'
        assert len(offset) == 3, 'Offset vector must have three components'

        if plane_origin is not None:
            assert len(
                plane_origin) == 3, 'Origin vector must have three components'

        if transform is not None:
            assert transform.shape == (
                4, 4), 'The transform matrix must be (4, 4)'

        if not self.load_mesh():
            PCG_ROOT_LOGGER.error(
                'Failed to load mesh, filename={}'.format(
                    self.filename))
            return False

        sliced_meshes = list()
        for mesh in self.get_meshes():
            geo = mesh.copy()

            if transform is not None:
                # Apply transformation matrix to mesh before sectioning it
                geo.apply_transform(transform)
                geo.apply_translation(offset)

            if plane_origin is None:
                plane_origin = geo.centroid

            sliced_meshes.append(
                trimesh.intersections.slice_mesh_plane(
                    geo, plane_normal, plane_origin))

        return sliced_meshes

    def _get_mesh_slice(self, z_limits=None, transform=None, offset=[0, 0, 0]):
        assert len(offset) == 3, 'Offset vector must have three components'

        if transform is not None:
            assert transform.shape == (
                4, 4), 'The transform matrix must be (4, 4)'

        if not self.load_mesh():
            PCG_ROOT_LOGGER.error(
                'Failed to load mesh, filename={}'.format(
                    self.filename))
            return False

        sliced_meshes = list()
        for mesh in self.get_meshes():
            geo = mesh.copy()

            if transform is not None:
                # Apply transformation matrix to mesh before sectioning it
                geo.apply_transform(transform)
                geo.apply_translation(offset)

            if z_limits is None:
                z_limits = geo.bounds[:, 2].flatten()

            plane_normal = np.array([0, 0, 1])

            # Create the selection box for the slice

            lower_mesh = trimesh.intersections.slice_mesh_plane(
                geo, plane_normal, [0, 0, z_limits[0]])

            if lower_mesh is None:
                return None

            upper_mesh = trimesh.intersections.slice_mesh_plane(
                lower_mesh, -plane_normal, [0, 0, z_limits[1]])

            sliced_meshes.append(upper_mesh)
        return sliced_meshes

    def get_meshes(self):
        self.load_mesh()
        meshes = list()
        if self._mesh is not None:
            if isinstance(self._mesh, trimesh.Scene):
                meshes = self._mesh.dump()
            elif isinstance(self._mesh, trimesh.Trimesh):
                meshes = [self._mesh]
            else:
                raise ValueError('Mesh object is not a valid trimesh object')

        if self._scale != [1, 1, 1]:
            # Apply scaling to the meshes
            scale_matrix = np.eye(4)
            for i in range(3):
                scale_matrix[i, i] = self._scale[i]

            for i in range(len(meshes)):
                new_mesh = meshes[i].copy()
                new_mesh.apply_transform(scale_matrix)
                meshes[i] = new_mesh
        return meshes

    def apply_transform(self, position, rot):
        self.load_mesh()
        meshes = self.get_meshes()

        if len(meshes) == 0:
            PCG_ROOT_LOGGER.warning('No meshes found')
            return None

        transformed_meshes = list()
        for mesh in self.get_meshes():
            mesh = mesh.copy()

            mesh.apply_transform(rot)
            mesh.apply_translation(position)
            transformed_meshes.append(mesh)

        return transformed_meshes

    def load_mesh(self):
        if self._mesh is None:
            if self._filename is None:
                msg = 'Mesh filename is required to load the mesh'
                PCG_ROOT_LOGGER.error(msg)
                raise ValueError(msg)

            if not os.path.isfile(self._filename):
                msg = 'Input mesh filename is invalid, filename={}'.format(
                    self._filename)
                PCG_ROOT_LOGGER.error(msg)
                raise ValueError(msg)

            entity = trimesh.load_mesh(self._filename)

            if isinstance(entity, trimesh.Scene):
                meshes = list(entity.dump())
                PCG_ROOT_LOGGER.info('# meshes={}, filename={}'.format(
                    len(meshes), self._filename))
                if len(meshes) == 1:
                    self._mesh = meshes[0]
                else:
                    self._mesh = entity
            else:
                self._mesh = entity

            PCG_ROOT_LOGGER.info(
                'Mesh successfully loaded from file, '
                'filename={}, # vertices={}'.format(
                    self._filename, self.n_vertices))
            return True
        else:
            return True

    def get_samples(self, count=1000):
        self.load_mesh()

        if self._mesh is None:
            return None

        try:
            n_vertices = self.n_vertices
            pnts = None
            for mesh in self.get_meshes():
                n_samples = int(count * mesh.vertices.shape[0] / n_vertices)
                if pnts is None:
                    pnts = mesh.sample(n_samples)
                else:
                    pnts = np.vstack((pnts, mesh.sample(n_samples)))
        except ValueError:
            pnts = self.vertices

        return pnts

    def compute_bounds(self):
        if not self.load_mesh():
            PCG_ROOT_LOGGER.error(
                'Cannot compute mesh bounds, filename={}'.format(
                    self.filename))
            return False

        self._bounds = dict(
            lower_x=None,
            upper_x=None,
            lower_y=None,
            upper_y=None,
            lower_z=None,
            upper_z=None)

        if self._mesh is not None:
            bounds = self._mesh.bounds
            self._bounds['lower_x'] = bounds[0, 0]
            self._bounds['upper_x'] = bounds[1, 0]

            self._bounds['lower_y'] = bounds[0, 1]
            self._bounds['upper_y'] = bounds[1, 1]

            self._bounds['lower_z'] = bounds[0, 2]
            self._bounds['upper_z'] = bounds[1, 2]
        return self._bounds

    def get_footprint_polygon(
            self,
            z_limits=None,
            use_global_frame=False,
            origin=None,
            plane_normal=[0, 0, 1],
            transform=None,
            offset=[0, 0, 0],
            use_bounding_box=False):
        assert len(
            plane_normal) == 3, 'Plane normal vector' \
            ' must have three components'
        assert np.sum(
            plane_normal) == 1, 'Plane normal vector must be a unit vector'
        assert len(offset) == 3, 'Offset vector must have three components'

        if origin is not None:
            assert len(origin) == 3, 'Origin vector must have three components'

        if transform is not None:
            assert transform.shape == (
                4, 4), 'The transform matrix must be (4, 4)'

        if not self.load_mesh():
            PCG_ROOT_LOGGER.error(
                'Cannot calculate footprint, filename={}'.format(
                    self.filename))
            return None

        footprint = None
        for m in self.get_meshes():
            mesh = m.copy()

            if transform is not None:
                # Apply transformation matrix to mesh before sectioning it
                mesh.apply_transform(transform)

            if offset is not None:
                mesh.apply_translation(offset)

            step = 0.01

            if z_limits is None:
                PCG_ROOT_LOGGER.info(
                    'Section mesh along the Z limit={},'
                    ' filename={}'.format(
                        mesh.bounds[:, 2].flatten(), self.filename))
                z_limits = mesh.bounds[:, 2].flatten()

                if (z_limits[1] - z_limits[0]) < step:
                    step = (z_limits[1] - z_limits[0]) / 10

                z_levels = np.arange(0, z_limits[1] - z_limits[0] + step, step)

                if origin is None:
                    origin = deepcopy(mesh.bounds[0, :])

            else:
                if z_limits[0] >= mesh.bounds[1, 2]:
                    PCG_ROOT_LOGGER.warning(
                        'Lower Z limit provided is outside of mesh range,'
                        ' no footprint for this range, min_z_limit={},'
                        ' max_z_bound={}'.format(
                            z_limits[0], mesh.bounds[1, 2]))
                    return None
                if z_limits[1] <= mesh.bounds[0, 2]:
                    PCG_ROOT_LOGGER.warning(
                        'Upper Z limit provided is outside of mesh range,'
                        ' no footprint for this range, max_z_limit={},'
                        ' min_z_bound={}'.format(
                            z_limits[1], mesh.bounds[0, 2]))
                    return None

                z_limits[0] = max(z_limits[0], mesh.bounds[0, 2])
                z_limits[1] = min(z_limits[1], mesh.bounds[1, 2])

                if np.abs(z_limits[1] - z_limits[0]) < 10 * step:
                    step = np.abs(z_limits[1] - z_limits[0]) / 10

                z_levels = np.arange(0, z_limits[1] - z_limits[0] + step, step)

                if origin is None:
                    origin = deepcopy(mesh.bounds[0, :])
                    origin[2] = z_limits[0]

            PCG_ROOT_LOGGER.info(
                'Sections will be acquired in the'
                ' following Z heights={}'.format(z_levels))

            polys = list()
            # List of lines for the rest of the forms found in the sections
            # that do not form a closed polygon
            lines = list()

            # Since trimesh computes the Path2D objects for the section
            # contours in a different coordinate system to the mesh
            # itself, store the lower bound to correct the position later
            fp_offset = [mesh.bounds[0, 0], mesh.bounds[0, 1]]

            if mesh.is_empty:
                PCG_ROOT_LOGGER.warning(
                    'Mesh is empty after slicing in interval={}'.format(
                        z_limits))
                return None

            if use_bounding_box:
                mesh = mesh.bounding_box_oriented

            try:
                closed_polys, combined_section = self._get_combined_sections(
                    mesh, origin, plane_normal, z_levels)

                if None in [closed_polys, combined_section]:
                    PCG_ROOT_LOGGER.warning(
                        'No slices found for mesh provided limits,'
                        ' z_limits={}, filename={}'.format(
                            z_limits, self._filename))
                    return None
            except ValueError as ex:
                PCG_ROOT_LOGGER.warning(
                    'Mesh sectioning failed, using the vertices'
                    ' bounding box instead, filename={}, message={}'.format(
                        self.filename, str(ex)))
                vertices = np.unique(mesh.vertices, axis=0)
                return MultiPoint(
                    [(vertices[i, 0], vertices[i, 1])
                        for i in range(vertices.shape[0])]).convex_hull

            # combined_section.show()
            # Retrieve all vertices, Path2D provides the index for the point
            vertices = combined_section.vertices

            # Get all remaining lines that are not included in any of the
            # polygons
            for trimesh_line in combined_section.entities:
                if trimesh_line.closed:
                    continue
                line = translate(LineString(
                    vertices[trimesh_line.points]), fp_offset[0], fp_offset[1])

                has_similar_line = False
                for l in lines:
                    if l.almost_equals(line, decimal=3):
                        has_similar_line = True
                        break
                if not has_similar_line:
                    lines.append(line)

            if footprint is None:
                footprint = deepcopy(closed_polys)
            else:
                footprint = unary_union([footprint] + closed_polys)
            PCG_ROOT_LOGGER.info(
                'Closed polys found in the sections,'
                ' area={}, filename={}'.format(
                    footprint.area, self.filename))

            # Create list of open polygons to be processed
            open_polys = list()

            if len(lines) > 0:
                PCG_ROOT_LOGGER.info(
                    'Processing remaining section lines into closed'
                    ' polygons, filename={}'.format(self.filename))

                def has_intersection(lines):
                    for i in range(len(lines)):
                        for j in range(len(lines)):
                            if i == j:
                                continue
                            line_i = lines[i]
                            line_j = lines[j]

                            if line_i.intersects(line_j):
                                return i, j
                    return None

                while has_intersection(lines) is not None:
                    i, j = has_intersection(lines)

                    line_i = lines[i]
                    line_j = lines[j]
                    if line_i.almost_equals(line_j, decimal=3):
                        new_line = line_i
                    else:
                        new_line = unary_union([line_i, line_j])

                    lines.remove(line_i)
                    lines.remove(line_j)

                    lines.append(new_line)

                for line in lines:
                    if isinstance(line, LineString) or line.is_empty:
                        continue

                    polys = list()
                    try:
                        polys = polys + list(polygonize(line))
                    except ValueError as ex:
                        PCG_ROOT_LOGGER.error(
                            'Could not polygonize lines, message={},'
                            ' filename={}'.format(
                                ex, self.filename))

                    try:
                        # Use Delaunay triangulation on the remaining
                        # intersecting lines
                        polys = polys + \
                            triangulate(MultiPoint(
                                self._multiline2points(line)))
                    except ValueError as ex:
                        PCG_ROOT_LOGGER.error(
                            'Could not triangulate lines, message={},'
                            ' filename={}'.format(
                                ex, self.filename))

                    if len(polys) == 0:
                        PCG_ROOT_LOGGER.error(
                            'Polygon list is empty, using convex'
                            ' hull of all lines,'
                            ' filename={}'.format(self.filename))
                        polys = [line.convex_hull]

                    for poly in polys:
                        if poly.is_empty or poly.area <= 1e-4 or \
                            not poly.is_valid or \
                                (not isinstance(poly, Polygon) and
                                    not isinstance(poly, MultiPolygon)):
                            continue

                        # Calculate the difference between the closed
                        # polygons and the new triangle to avoid the
                        # polygon's interior
                        try:
                            poly = poly.difference(closed_polys)
                        except BaseException:
                            continue

                        if poly.is_empty or poly.area <= 1e-4 or \
                            not poly.is_valid or \
                                (not isinstance(poly, Polygon) and
                                    not isinstance(poly, MultiPolygon)):
                            continue

                        # Use ray tracing to see if the polygon's
                        # centroid crosses the mesh at some point
                        ray_origins = np.array(
                            [[
                                poly.centroid.x,
                                poly.centroid.y,
                                mesh.bounds[0, 2]
                            ]])
                        ray_directions = np.array([[0, 0, 1]])

                        locations, index_ray, index_tri = \
                            mesh.ray.intersects_location(
                                ray_origins=ray_origins,
                                ray_directions=ray_directions)

                        if len(locations) > 0:
                            open_polys.append(poly.buffer(0.001))

            footprint = unary_union([footprint] + open_polys)
            # Dilate and erode the polygon to get rid of small gaps
            footprint = footprint.buffer(0.005).buffer(-0.005)

            PCG_ROOT_LOGGER.info(
                'Removing internal polygons from '
                'footprint, filename={}'.format(
                    self.filename))
            # Remove interior polygons
            if isinstance(footprint, Polygon):
                for interior_poly in footprint.interiors:
                    interior_poly = Polygon(interior_poly)
                    footprint = footprint.union(interior_poly)
            elif isinstance(footprint, MultiPolygon):
                for geo in footprint.geoms:
                    for interior_poly in geo.interiors:
                        interior_poly = Polygon(interior_poly)
                        geo = geo.union(interior_poly)
            PCG_ROOT_LOGGER.info(
                'Footprint final area, area={}, filename={}'.format(
                    footprint.area, self.filename))

        return footprint

    def show(self):
        if not self.load_mesh():
            return False
        self._mesh.show()
        return True

    def show_section(self, origin=None, plane_normal=[0, 0, 1], show_3d=True):
        if not self.load_mesh():
            PCG_ROOT_LOGGER.error('Cannot show section')
            return False

        if origin is not None:
            assert len(origin) == 3, 'Origin vector must have three components'
        else:
            origin = self._mesh.centroid

        assert len(plane_normal) == 3, 'Plane normal vector' \
            ' must have three components'
        assert np.sum(plane_normal) == 1, 'Plane normal vector' \
            ' must be a unit vector'

        slices = list()
        for mesh in self.get_meshes():
            s = mesh.section(plane_origin=origin, plane_normal=plane_normal)
            if not show_3d:
                s, to_3D = s.to_planar()
            if s is not None:
                slices.append(s)
        scene = trimesh.Scene(slices)
        scene.show()

    def show_footprint(
            self,
            z_limits=None,
            use_global_frame=False,
            origin=None,
            plane_normal=[0, 0, 1],
            transform=None,
            offset=[0, 0, 0],
            figsize=(10, 8),
            alpha=0.5,
            ax=None,
            show=True):
        try:
            from descartes.patch import PolygonPatch
        except ImportError:
            PCG_ROOT_LOGGER.warning('Install descartes to plot the footprint')
            return None

        try:
            from matplotlib import pyplot
        except ImportError:
            PCG_ROOT_LOGGER.warning('Install matplotlib to plot the footprint')
            return None

        if ax is None:
            fig = pyplot.figure(figsize=figsize)
            ax = fig.add_subplot(111)

        footprint = self.get_footprint_polygon(
            z_limits=z_limits,
            use_global_frame=False,
            origin=None,
            plane_normal=[0, 0, 1],
            transform=None,
            offset=[0, 0, 0])

        patch = PolygonPatch(
            footprint,
            facecolor='blue',
            edgecolor='black',
            alpha=0.5,
            zorder=2)
        ax.add_patch(patch)

        if show:
            x_lim = np.abs(self._mesh.bounds[:, 0]).max() + 0.5
            y_lim = np.abs(self._mesh.bounds[:, 1]).max() + 0.5

            ax.axis('equal')
            ax.set_xlim(-x_lim, x_lim)
            ax.set_ylim(-y_lim, y_lim)
            ax.grid(True)

        return ax

    def to_sdf(self, uri_type=None, mesh_filename=None, model_folder=None,
               copy_resources=False):
        if self._filename is None:
            from ...utils import generate_random_string, PCG_RESOURCES_ROOT_DIR
            PCG_ROOT_LOGGER.info('Exporting mesh to file')
            if mesh_filename:
                PCG_ROOT_LOGGER.info('Mesh filename: {}'.format(mesh_filename))
            if model_folder:
                PCG_ROOT_LOGGER.info('Model folder: {}'.format(model_folder))
            if model_folder is not None:
                if not os.path.isdir(model_folder):
                    PCG_ROOT_LOGGER.warning(
                        'Input resources folder to store meshes'
                        ' does not exist, using the default {}, '
                        'dir={}'.format(PCG_RESOURCES_ROOT_DIR, model_folder))
                    model_folder = PCG_RESOURCES_ROOT_DIR
                folder = os.path.join(model_folder, 'meshes')

                if mesh_filename is None:
                    filename = generate_random_string(10)
                else:
                    filename = mesh_filename
            else:
                folder = os.path.join(PCG_RESOURCES_ROOT_DIR, 'meshes')
                filename = generate_random_string(10)

            if not os.path.isdir(folder):
                os.makedirs(folder)

            # Store the mesh as STL per default
            self.export_mesh(filename, folder, format='stl')

            # Update the URI to the mesh file
            self._uri = Path(os.path.join(folder, filename + '.stl'))
            self._filename = self._uri.absolute_uri
            PCG_ROOT_LOGGER.info('Mesh stored at {}'.format(self._filename))
        elif model_folder is not None and copy_resources:
            PCG_ROOT_LOGGER.info(
                'Copying mesh resource <{}> to model folder <{}>'.format(
                    self._filename, model_folder))
            from shutil import copyfile
            if not os.path.isdir(model_folder):
                PCG_ROOT_LOGGER.warning(
                    'Input resources folder to store meshes'
                    ' does not exist, meshes will not be copied'
                    'dir={}'.format(model_folder))
            else:
                folder = os.path.join(model_folder, 'meshes')
                if not os.path.isdir(folder):
                    os.makedirs(folder)

                if mesh_filename is None:
                    mesh_filename = os.path.basename(self._uri.absolute_uri)
                old_filename = self._uri.absolute_uri
                copyfile(
                    old_filename,
                    os.path.join(folder, mesh_filename))
                self._uri = Path(os.path.join(folder, mesh_filename))
                self._filename = self._uri.absolute_uri
                PCG_ROOT_LOGGER.info(
                    'Mesh file was copied from {} to {}'.format(
                        old_filename, self._uri.absolute_uri))

        assert self.filename is not None, \
            'Mesh has not filename to fill the SDF element'

        mesh = create_sdf_element('mesh')

        PCG_ROOT_LOGGER.info(self._uri.model_uri)

        if uri_type is None:
            if self._uri.model_uri is not None:
                mesh.uri = self._uri.model_uri
            elif self._uri.file_uri is not None:
                mesh.uri = self._uri.file_uri
        else:
            if uri_type == 'file':
                mesh.uri = self._uri.file_uri
            elif uri_type == 'model':
                mesh.uri = self._uri.model_uri
            elif uri_type == 'package':
                mesh.uri = self._uri.package_uri
            else:
                msg = 'Invalid type of URI for SDF export'
                PCG_ROOT_LOGGER.error(msg)
                raise ValueError(msg)
        mesh.scale = self._scale
        return mesh

    def export_mesh(self, filename=None, folder=None, format='stl'):
        if not self.load_mesh():
            PCG_ROOT_LOGGER.error('Cannot show section')
            return None
        export_formats = ['stl', 'dae', 'obj', 'json']
        if format not in export_formats:
            PCG_ROOT_LOGGER.error(
                'Invalid mesh export format, options={}'.format(
                    export_formats))
            return None
        if not os.path.isdir(folder):
            PCG_ROOT_LOGGER.error(
                'Export folder does not exist, provided={}'.format(folder))
            return None

        mesh_filename = os.path.join(folder, filename + '.' + format)
        trimesh.exchange.export.export_mesh(
            self._mesh,
            mesh_filename,
            file_type=format if format != 'stl' else 'stl_ascii')
        return mesh_filename

    def plane_fit(self):
        if not self.load_mesh():
            PCG_ROOT_LOGGER.error('Cannot show section')
            return None

        center, normal = trimesh.points.plane_fit(self.vertices)
        return center, normal

    def major_axis(self):
        if not self.load_mesh():
            PCG_ROOT_LOGGER.error('Cannot show section')
            return None
        axis = trimesh.points.major_axis(self.vertices)
        return axis
