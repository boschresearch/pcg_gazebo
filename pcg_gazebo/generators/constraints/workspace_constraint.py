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
from .constraint import Constraint
from ...utils import is_string
import collections
import random
from shapely.geometry import Polygon, LineString, Point, MultiPoint


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
    _GEOMETRIES = [
        'line',
        'area',
        'volume',
        'multi_line',
        'multi_point',
        'circle']

    def __init__(self, geometry=None, frame='world', holes=None):
        Constraint.__init__(self)

        assert isinstance(
            geometry, dict), 'Geometry specification must be a dictionary'
        assert 'type' in geometry, 'No geometry type was specified'
        assert geometry['type'] in self._GEOMETRIES, \
            'Invalid workspace geometry, options={}'.format(self._GEOMETRIES)
        assert is_string(frame), \
            'Input frame name must be a string'

        self._geometry_type = geometry['type']
        self._holes = list()
        self._geometry = None
        self._geometry = self.generate_geometry(
            self._geometry_type, geometry['description'])

        if holes is not None:
            for hole in holes:
                self._holes.append(self.generate_geometry(**hole))

        self._frame = frame

    def __str__(self):
        """Return formatted string."""
        msg = 'Workspace constraint\n'
        msg += '\t - Type of geometry: {}\n'.format(self._geometry_type)
        return msg

    def generate_geometry(self, type, description):
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
        assert isinstance(description, dict), \
            'Description information must be a dictionary'
        if 'points' in description:
            return self._generate_geometry_from_points(type, **description)
        elif type == 'circle' and \
                'center' in description and 'radius' in description:
            return self._generate_circle(**description)
        else:
            raise NotImplementedError()

    def _generate_geometry_from_points(self, geometry_type, points):
        """For geometries of type `line` and `area`, return the `shapely`
        entity describing a line or a polygon, respectively.

        > *Input arguments*

        * `geometry_type` (*type:* `str`): Type of geometry,
         options are `line` and `area`
        * `points` (*type:* `list`): List of 3D vectors
         representing the vertices of the area or the
         points on the line
        """
        if geometry_type == 'line':
            # Only x and y coordinates are considered for line
            geometry = LineString([(x[0], x[1]) for x in points])
        elif geometry_type == 'area':
            # Only x and y coordinates are considered for polygon area
            geometry = Polygon([(x[0], x[1]) for x in points])
        else:
            raise NotImplementedError()
        return geometry

    def _generate_circle(self, center, radius):
        """Return a `shapely.Polygon` representing the circle
        described by `center` and `radius`.

        > *Input arguments*

        * `center` (*type:* `list`): 2D or 3D vector
         representing the center of the circle. For 2D
         points, the Z component is assumed to be 0.
        * `radius` (*type:* `float`): Radius of the circle
        """
        assert len(center) in [
            2, 3], 'Center of circle must have 2 or 3 elements'
        assert radius > 0, 'Radius must be greater than zero'
        return Point(*center).buffer(radius)

    def get_bounds(self):
        """Return the polygon bounds"""
        return self._geometry.bounds

    def get_random_position(self):
        """Return a random position that belongs to the workspace"""
        if self._geometry_type in ['area', 'circle']:
            geo = self.get_geometry()
            min_x, min_y, max_x, max_y = geo.bounds
            pnt = Point(
                random.uniform(min_x, max_x),
                random.uniform(min_y, max_y))
            while not geo.contains(pnt):
                pnt = Point(
                    random.uniform(min_x, max_x),
                    random.uniform(min_y, max_y))
            return pnt
        else:
            return None

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
        return geo.contains(vertices.convex_hull)

    def get_geometry(self):
        """Return the workspace geometry"""
        geometry = self._geometry
        for geo in self._holes:
            geometry = geometry.difference(geo)
        return geometry
