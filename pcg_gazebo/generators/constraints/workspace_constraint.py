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
import numpy as np
import trimesh
from copy import deepcopy
from .constraint import Constraint
from ... import random
from .. import shapes
from ...utils import is_string, get_random_point_from_shape, \
    is_array
from ...transformations import quaternion_matrix
from ...simulation.properties import Pose
import collections
from shapely.geometry import Polygon, LineString, Point, \
    MultiPoint, MultiPolygon, MultiLineString
from shapely.affinity import affine_transform


class WorkspaceConstraint(Constraint):
    """Class that represents the spatial workspace where models are allowed in.
    The `geometry` input is a `dict` containing all the arguments necessary to
    generate the workspace geometry. For now, only 2D workspaces are supported.
    The `holes` input is a list of `dict` with the same geometry description of
    the input `geometry` and describe exclusion areas inside the workspace.

    The supported geometry inputs to represent a workspace are

    * `area`

    ```python
    geometry = dict(
        type='area'
        description=dict(
            points=[
               [0, 0, 0],
               [0, 1, 1],
               ...
               ]  # List of 3D points that describe the
                    vertices of the plane area
        )
    )
    ```

    * `line`

    ```python
    geometry=dict(
        type='line',
        description=dict(
            points=[
               [0, 0, 0],
               [0, 1, 1],
               ...
               ]  # List of 3D points that describe the line
        )
    )
    ```

    * `circle`

    ```python
    geometry=dict(
        type='circle'
        description=dict(
            radius=0.0, # Radius of the circle
            center=[0, 0, 0] # Center of the circle as a 3D point
        )
    )
    ```

    **Others are still not implemented**

    > *Attributes*

    * `LABEL` (*type:* `str`, *value:* `workspace`): Name of
    the constraint class
    * `GEOMETRIES` (*type:* `list`): List of input geometries
    that can be used to set a workspace

    > *Input arguments*

    * `geometry` (*type:* `dict`, *default:* `None`): Input
    arguments of the geometry to be generated
    * `frame` (*type:* `str`, *default:* `'world'`): Name of
    the frame of reference of the workspace (**not implemented**)
    * `holes` (*type:* `dict`, *default:* `None`): Geometries
    that represent exclusion areas inside the workspace
    """
    _LABEL = 'workspace'

    def __init__(self, geometry_type=None, frame='world', holes=None,
                 pose=None, **kwargs):
        Constraint.__init__(self)

        assert is_string(frame), \
            'Input frame name must be a string'

        self._geometry_type = geometry_type
        self._holes = list()
        self._geometry = None
        self._geometry = self.generate_geometry(
            self._geometry_type, **kwargs)

        if holes is not None:
            for hole in holes:
                self._holes.append(self.generate_geometry(**hole))

        self._frame = frame
        self._pose = Pose()
        if pose is not None:
            self.pose = pose

    def __eq__(self, other):
        if not isinstance(other, WorkspaceConstraint):
            return False
        if other._LABEL != self._LABEL:
            return False
        if self._geometry_type != other._geometry_type:
            return False
        if self._geometry != other._geometry:
            return False
        if len(self._holes) != len(other._holes):
            return False
        for hole in self._holes:
            if hole not in other._holes:
                return False
        return True

    def __str__(self):
        """Return formatted string."""
        msg = 'Workspace constraint\n'
        msg += '\t - Type of geometry: {}\n'.format(self._geometry_type)
        return msg

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, vec):
        if isinstance(vec, Pose):
            self._pose = vec
        else:
            assert is_array(vec), \
                'Input pose vector must be iterable'
            assert len(vec) == 6 or len(vec) == 7, \
                'Pose must be given as position and Euler angles (x, y, z, ' \
                'roll, pitch, yaw) or position and quaternions (x, y, z, ' \
                'qx, qy, qz, qw)'
            for item in vec:
                assert isinstance(item, float) or isinstance(item, int), \
                    'All elements in pose vector must be a float or an integer'

            self._pose = Pose(pos=vec[0:3], rot=vec[3::])

    def generate_geometry(self, type, **kwargs):
        """Generate a `shapely` entity according to the geometry description
        provided. The input `type` containts the name of the geometry to
        be generated and the necessary arguments must be provided in the `dict`
        input `description`.

        Possible geometries according to the different input
        values in `type` are:

        * `area`

        ```python
        description=dict(
           points=[
               [0, 0, 0],
               [0, 1, 1],
               ...
               ]  # List of 3D points that describe the
                    vertices of the plane area
        )
        ```

        * `line`

        ```python
        description=dict(
           points=[
               [0, 0, 0],
               [0, 1, 1],
               ...
               ]  # List of 3D points that describe the line
        )
        ```

        * `circle`

        ```python
        description=dict(
          center=[-6.8, -6.8, 0] # Center of the circle
          radius=0.2  # Radius of the circle
        )
        ```

        **Others are still not implemented**

        > *Input arguments*

        * `type` (*type:* `str`): Geometry type. Options
         are: `line`, `area`, `volume`, `multi_line`, `multi_point`, `circle`
        * `description` (*type:* `dict`): Arguments to describe the geometry
        """
        if type == 'area' and 'points' in kwargs:
            return MultiPoint(
                [(x[0], x[1]) for x in kwargs['points']]).convex_hull
        elif type == 'line' and 'points' in kwargs:
            line = LineString([(x[0], x[1]) for x in kwargs['points']])
            if 'buffer' in kwargs:
                assert kwargs['buffer'] > 0, \
                    'Buffer around line must be greater than 0'
                return line.buffer(kwargs['buffer'])
            else:
                return line
        elif type == 'multipoint' and 'points' in kwargs:
            points = MultiPoint(
                [(x[0], x[1]) for x in kwargs['points']])
            if 'buffer' in kwargs:
                assert kwargs['buffer'] > 0, \
                    'Buffer around line must be greater than 0'
                return points.buffer(kwargs['buffer'])
            else:
                return points
        elif type == 'multiline' and 'lines' in kwargs:
            lines = MultiLineString(kwargs['lines'])
            if 'buffer' in kwargs:
                assert kwargs['buffer'] > 0, \
                    'Buffer around line must be greater than 0'
                return lines.buffer(kwargs['buffer'])
            else:
                return lines
        elif type == 'circle' and \
                'center' in kwargs and 'radius' in kwargs:
            return shapes.circle(**kwargs)
        elif type == 'polygon' and 'polygon' in kwargs:
            assert isinstance(kwargs['polygon'], (Polygon, MultiPolygon))
            assert not kwargs['polygon'].is_empty, 'Polygon is empty'
            assert kwargs['polygon'].area > 0, 'Polygon area is zero'
            return kwargs['polygon']
        elif type == 'box':
            assert 'size' in kwargs
            return trimesh.creation.box(extents=kwargs['size'])
        elif type == 'sphere':
            assert 'radius' in kwargs
            assert kwargs['radius'] > 0
            return trimesh.creation.icosphere(
                radius=kwargs['radius'])
        elif type == 'cylinder':
            assert 'radius' in kwargs
            assert kwargs['radius'] > 0
            assert 'length' in kwargs
            assert kwargs['length'] > 0
            return trimesh.creation.cylinder(
                radius=kwargs['radius'],
                height=kwargs['length'])
        else:
            raise NotImplementedError(
                'Invalid geometry type, provided={}'.format(type))

    def _apply_transform(self, geometry):
        mat = quaternion_matrix(self._pose.quat)
        if isinstance(geometry, trimesh.base.Trimesh):
            geo = geometry.copy()

            geo.apply_transform(mat)
            geo.apply_translation(self._pose.position)
            return geo
        else:
            transform = np.zeros(12)
            transform[0:3] = mat[0, 0:3].flatten()
            transform[3:6] = mat[1, 0:3].flatten()
            transform[6:9] = mat[2, 0:3].flatten()
            transform[9:12] = self._pose.position
            return affine_transform(geometry, transform)

    def _compute_random_point_on_geometry(self, geometry):
        if isinstance(geometry, (Polygon, MultiPolygon)):
            return Point(
                get_random_point_from_shape(self.get_geometry()))
        elif isinstance(geometry, LineString):
            idx = random.randint(1, len(geometry.coords))
            start_point = np.array(geometry.coords[idx - 1])
            vec = np.array(geometry.coords[idx]) - \
                np.array(geometry.coords[idx - 1])
            random_point = start_point + random.rand() * vec
            return Point(random_point)
        else:
            raise NotImplementedError('Invalid geometry type={}'.format(
                type(geometry)))

    def _compute_random_point_in_mesh(self, mesh):
        min_x = mesh.bounds[0, 0]
        max_x = mesh.bounds[1, 0]

        min_y = mesh.bounds[0, 1]
        max_y = mesh.bounds[1, 1]

        min_z = mesh.bounds[0, 2]
        max_z = mesh.bounds[1, 2]
        pnt = [
            random.uniform(min_x, max_x),
            random.uniform(min_y, max_y),
            random.uniform(min_z, max_z)
        ]
        while not mesh.contains([pnt]):
            pnt = [
                random.uniform(min_x, max_x),
                random.uniform(min_y, max_y),
                random.uniform(min_z, max_z)
            ]
        return Point(pnt)

    def get_bounds(self):
        """Return the polygon bounds"""
        return self._geometry.bounds

    def get_random_position(self):
        """Return a random position that belongs to the workspace"""
        geometry = self.get_geometry()
        if isinstance(geometry, (Polygon, LineString)):
            return self._compute_random_point_on_geometry(geometry)
        elif isinstance(geometry, (MultiLineString, MultiPolygon)):
            idx = random.randint(0, len(geometry.geoms))
            return self._compute_random_point_on_geometry(
                geometry.geoms[idx])
        elif isinstance(geometry, MultiPoint):
            idx = random.randint(0, len(geometry.geoms))
            return geometry.geoms[idx]
        else:
            return self._compute_random_point_in_mesh(geometry)

    def contains_point(self, point):
        """Return True if `point` is part of the workspace.

        > *Input arguments*

        * `point` (*type:* `list` or `numpy.ndarray`): 2D point
        """
        assert isinstance(point, collections.Iterable), \
            'Invalid list of points'
        point = list(point)
        geo = self.get_geometry()
        # Only planar points are checked now
        pnt = Point(point[0], point[1])
        return geo.contains(pnt)

    def contains_points(self, points):
        assert isinstance(points, collections.Iterable), \
            'Invalid list of points'
        points = MultiPoint(points)
        geo = self.get_geometry()
        return geo.contains(points)

    def contains_polygons(self, polygons):
        """Return True if polygons in the `polygons` list are part of the workspace.

        > *Input arguments*

        * `polygons` (*type:* list of `shapely.Polygon`): List of polygons
        """
        assert isinstance(polygons, collections.Iterable), \
            'Invalid list of polygons'
        merged_poly = None
        geo = self.get_geometry()
        for poly in polygons:
            if merged_poly is None:
                merged_poly = geo.union(poly)
            else:
                merged_poly = merged_poly.union(poly)
        return merged_poly.area == geo.area

    def contains_mesh(self, mesh):
        vertices = MultiPoint(mesh.vertices[:, 0:2].tolist())
        geo = self.get_geometry()

        if isinstance(geo, (Polygon, MultiPolygon)):
            return geo.contains(vertices.convex_hull)
        elif isinstance(geo, (LineString, MultiLineString)):
            convex_hull = vertices.convex_hull
            return convex_hull.intersects(geo) or convex_hull.contains(geo)
        elif isinstance(geo, (MultiPoint)):
            convex_hull = vertices.convex_hull
            for point in geo.geoms:
                if convex_hull.contains(point):
                    return True
            return False
        elif isinstance(trimesh.base.Trimesh):
            return geo.contains(mesh.vertices)

    def add_hole(self, type, **kwargs):
        self._holes.append(self.generate_geometry(type, **kwargs))

    def get_geometry(self):
        """Return the workspace geometry"""
        geometry = deepcopy(self._geometry)
        for geo in self._holes:
            geometry = geometry.difference(geo)
        return self._apply_transform(geometry)
