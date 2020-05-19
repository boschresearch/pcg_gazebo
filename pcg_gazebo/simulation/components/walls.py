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
from copy import deepcopy
from shapely.geometry import Polygon, MultiPolygon, LineString, \
    MultiLineString
from shapely.ops import unary_union
from ..model import SimulationModel
from ...generators.shapes import rectangle, random_rectangle, \
    random_points_to_triangulation, triangulate_points, \
    random_rectangles, random_orthogonal_lines
from ...generators.mesh import extrude
from ...visualization import plot_shapely_geometry
from ...log import PCG_ROOT_LOGGER


class Walls(SimulationModel):
    _CAP_STYLES = ['round', 'flat', 'square']
    _JOIN_STYLES = ['round', 'mitre', 'bevel']

    def __init__(self, height, thickness=0.1, geometries=None,
                 model_name='wall', cap_style='square', join_style='mitre',
                 ):
        super(Walls, self).__init__(name=model_name)

        assert height > 0, "Walls height must be greater than zero"
        assert thickness > 0, "Walls thickness is greater than zero"

        # Walls height
        self._height = height
        # Walls thickness
        self._thickness = thickness
        # Add the list of polygons describing the floor plan
        self._geometries = list()
        if geometries is not None:
            if not isinstance(geometries, list):
                self._geometries = [geometries]
            else:
                self._geometries = geometries

        # Polygons representing interior spaces for closed polygons
        self._floorplan_geometries = None
        self._floorplan_lines = None
        self._interior_polygons = dict()
        self._wall_polygons = dict()

        # Shapely operation settings
        # Cap style for buffering
        assert cap_style in self._CAP_STYLES, \
            'Invalid cap style, options are {}'.format(self._CAP_STYLES)
        self._cap_style = cap_style
        # Join style for buffering
        assert join_style in self._JOIN_STYLES, \
            'Invalid join style, options are {}'.format(self._JOIN_STYLES)
        self._join_style = join_style

        # Set wall model as static
        self.static = True
        self.update_walls()

    @property
    def geometries(self):
        return self._geometries

    @property
    def floors(self):
        return self._interior_polygons

    @property
    def wall_polygons(self):
        return self._wall_polygons

    @staticmethod
    def from_geometry(geometries, height, thickness, model_name='walls',
                      merge_geometries=False):
        assert geometries is not None, 'Geometry input cannot be None'
        if merge_geometries and isinstance(geometries, list):
            return Walls(
                height=height,
                thickness=thickness,
                geometries=unary_union(geometries),
                model_name=model_name)
        else:
            return Walls(
                height=height,
                thickness=thickness,
                geometries=geometries,
                model_name=model_name)

    @staticmethod
    def from_random_points(
            n_points=10,
            x_min=-10,
            x_max=10,
            y_min=-10,
            y_max=10,
            height=2.0,
            thickness=0.1,
            model_name='walls'):
        polygon = random_points_to_triangulation(
            n_points, x_min, x_max, y_min, y_max)
        model = Walls(
            height=height,
            thickness=thickness,
            model_name=model_name,
            geometries=polygon
        )

        return model

    @staticmethod
    def from_rectangle(x_min=-10, x_max=10, y_min=-10, y_max=10, height=2.0,
                       thickness=0.1, model_name='walls'):
        assert x_min < x_max, 'x_min must be lower than x_max'
        assert y_min < y_max, 'y_min must be lower than y_max'
        polygon = rectangle(
            (x_max - x_min) / 2. + x_min,
            (y_max - y_min) / 2. + y_min,
            (x_max - x_min),
            (y_max - y_min))

        model = Walls(
            height=height,
            thickness=thickness,
            model_name=model_name,
            geometries=polygon
        )
        return model

    @staticmethod
    def from_random_rectangle(
            x_center=0,
            y_center=0,
            delta_x_min=2,
            delta_x_max=15,
            delta_y_min=2,
            delta_y_max=15,
            height=2.0,
            thickness=0.1,
            model_name='walls'):
        model = Walls(
            height=height,
            thickness=thickness,
            model_name=model_name,
            geometries=random_rectangle(
                x_center, y_center, delta_x_min, delta_x_max,
                delta_y_min, delta_y_max)
        )
        return model

    @staticmethod
    def from_random_rectangles(
            n_rect=5,
            x_center_min=-10,
            x_center_max=10,
            y_center_min=-10,
            y_center_max=10,
            delta_x_min=2,
            delta_x_max=10,
            delta_y_min=2,
            delta_y_max=10,
            delete_interiors=False,
            height=2.0,
            thickness=0.1,
            model_name='walls'):
        polygon = random_rectangles(
            n_rect,
            x_center_min,
            x_center_max,
            y_center_min,
            y_center_max,
            delta_x_min,
            delta_x_max,
            delta_y_min,
            delta_y_max,
            delete_interiors)
        model = Walls(
            height=height,
            thickness=thickness,
            model_name=model_name,
            geometries=polygon
        )
        return model

    @staticmethod
    def from_random_orthogonal_lines(
            n_lines=5,
            x_min=-10,
            x_max=10,
            y_min=-10,
            y_max=10,
            line_min_length=1,
            line_max_length=15,
            height=2.0,
            thickness=0.1,
            model_name='walls'):
        lines = random_orthogonal_lines(
            n_lines,
            x_min,
            x_max,
            y_min,
            y_max,
            line_min_length,
            line_max_length)
        model = Walls(
            height=height,
            thickness=thickness,
            model_name=model_name,
            geometries=lines
        )
        return model

    def add_rectangle(self, x_min=-10, x_max=10,
                      y_min=-10, y_max=10):
        assert x_min < x_max, 'x_min must be lower than x_max'
        assert y_min < y_max, 'y_min must be lower than y_max'

        self._geometries.append(
            rectangle(
                (x_max - x_min) / 2. + x_min,
                (y_max - y_min) / 2. + y_min,
                (x_max - x_min),
                (y_max - y_min)))
        self.update_walls()

    def add_polygon(self, geometry):
        if isinstance(geometry, list):
            # Create a polygon from input coordinates
            poly = Polygon(geometry)
        elif isinstance(geometry, (Polygon, MultiPolygon)):
            poly = geometry
        else:
            PCG_ROOT_LOGGER.warning(
                'Input must be a list of (X,Y) coordinates'
                'defining a polygon, a shapely.Polygon or '
                'a shapely.MultiPolygon object')
            return False
        if not self.is_geometry_duplicate(poly):
            self._geometries.append(poly)
            self.update_walls()
            return True
        else:
            PCG_ROOT_LOGGER.warning('Geometry already exists in floorplan')
            return False

    def add_points(self, points):
        geometry = triangulate_points(points)
        if not self.is_geometry_duplicate(geometry):
            self._geometries.append(geometry)
            self.update_walls()
            return True
        else:
            PCG_ROOT_LOGGER.warning('Geometry already exists in floorplan')
            return False

    def add_lines(self, lines):
        if isinstance(lines, list):
            line = LineString(lines)
        elif isinstance(lines, (LineString, MultiLineString)):
            line = lines
        else:
            PCG_ROOT_LOGGER.warning(
                'Input must be a list of (X,Y) coordinates'
                ' or a shapely.MultiPoint object')
            return False
        if not self.is_geometry_duplicate(line):
            self._geometries.append(line)
            self.update_walls()
            return True
        else:
            PCG_ROOT_LOGGER.warning('Geometry already exists in floorplan')
            return False

    def is_geometry_duplicate(self, geometry):
        for geo in self._geometries:
            if geo.equals(geometry):
                return True
        return False

    def create_wall(self, geometry):
        wall_polygons = list()
        if isinstance(geometry, LineString):
            wall_polygons.append(geometry.buffer(
                self._thickness,
                cap_style=self._CAP_STYLES.index(self._cap_style) + 1,
                join_style=self._JOIN_STYLES.index(self._join_style) + 1))
        elif isinstance(geometry, MultiLineString):
            for geo in geometry.geoms:
                wall_polygons.append(geo.buffer(
                    self._thickness,
                    cap_style=self._CAP_STYLES.index(self._cap_style) + 1,
                    join_style=self._JOIN_STYLES.index(self._join_style) + 1))
        else:
            outer_wall = geometry.buffer(
                self._thickness / 2.,
                cap_style=self._CAP_STYLES.index(self._cap_style) + 1,
                join_style=self._JOIN_STYLES.index(self._join_style) + 1)
            inner_wall = geometry.buffer(
                -self._thickness / 2.,
                cap_style=self._CAP_STYLES.index(self._cap_style) + 1,
                join_style=self._JOIN_STYLES.index(self._join_style) + 1)

            # To get nice square corners, use the difference between
            # the outer and inner dilated polygons
            wall_polygons.append(outer_wall.difference(inner_wall))

        for poly in wall_polygons:
            wall_mesh = extrude(
                poly,
                height=self._height,
                extrude_boundaries=False)

            i = 0
            name = 'wall_{}'.format(i)
            while name in self.links:
                i += 1
                name = 'wall_{}'.format(i)

            self._wall_polygons[name] = poly

            # Add the link to the new wall
            self.add_link(
                visual_mesh=wall_mesh,
                collision_mesh=wall_mesh,
                use_approximated_collision=False,
                approximated_collision_model=False,
                visual_mesh_scale=[
                    1,
                    1,
                    1],
                collision_mesh_scale=[
                    1,
                    1,
                    1],
                name=name,
                mass=0,
                inertia=None,
                use_approximated_inertia=False,
                pose=[
                    wall_mesh.centroid[0],
                    wall_mesh.centroid[1],
                    wall_mesh.centroid[2],
                    0,
                    0,
                    0])

    def update_walls(self):
        if self._geometries is None or len(self._geometries) == 0:
            PCG_ROOT_LOGGER.warning(
                'Walls model <{}> has no polygons '
                'to describe floorplan'.format(
                    self.name))
            return False
        if not isinstance(self._geometries, list):
            PCG_ROOT_LOGGER.warning(
                'Floor plan polygons must be provided as a list')
            return False
        for geo in self._geometries:
            assert isinstance(
                geo,
                (Polygon, MultiPolygon,
                 LineString, MultiLineString)), 'Invalid geometry type'

        self._links = dict()

        # Merge geometries together
        self._floorplan_geometries = list()
        for geo in self._geometries:
            if len(self._floorplan_geometries) == 0:
                self._floorplan_geometries.append(geo)
            else:
                # Compute only the space where there is no
                # overlap with the existent floorplan (only full
                # polygons or multi polygons)
                combined_geometry = unary_union(
                    [fg for fg in self._floorplan_geometries
                        if isinstance(fg, (Polygon, MultiPolygon))])
                if combined_geometry.crosses(geo):
                    diff = geo.difference(combined_geometry)
                    if not diff.is_empty:
                        self._floorplan_geometries.append(diff)
                else:
                    self._floorplan_geometries.append(geo)

        self._floorplan_lines = None
        for geo in self._floorplan_geometries:
            if isinstance(geo, (Polygon, MultiPolygon)):
                lines = geo.boundary
            elif isinstance(geo, (LineString, MultiLineString)):
                lines = geo
            else:
                continue

            if self._floorplan_lines is None:
                self._floorplan_lines = deepcopy(lines)
            else:
                self._floorplan_lines = self._floorplan_lines.union(lines)

        self.create_wall(self._floorplan_lines)

        walls = unary_union(list(self._wall_polygons.values()))
        convex_hull = walls.convex_hull.buffer(0.1)

        self._interior_polygons = dict()
        diff = convex_hull.difference(walls)
        if hasattr(diff, 'geoms'):
            floors = list(diff.geoms)
            if len(floors) > 1:
                # In case only one geometry is found, the wall geometry
                # is only made of open floorplans
                i = 0
                for geo in list(convex_hull.difference(walls).geoms):
                    if not convex_hull.boundary.touches(geo):
                        name = 'floor_{}'.format(i)
                        i += 1
                        self._interior_polygons[name] = geo

        for tag in self._links:
            for visual in self._links[tag].visuals:
                visual.cast_shadows = False
                visual.set_material_script('Gazebo/DarkGrey')

        return True

    def plot_floorplan(
            self,
            fig=None,
            ax=None,
            fig_width=20,
            fig_height=15,
            use_matplotlib=True,
            plot_reference=False,
            plot_interior=True,
            alpha=1.0,
            line_width=2,
            line_style='solid',
            reference_geo_color='tab:blue',
            wall_geo_color='tab:gray',
            interior_geo_color='tab:pink'):
        for tag in self._wall_polygons:
            fig, ax = plot_shapely_geometry(
                fig=fig,
                ax=ax,
                color=wall_geo_color,
                use_matplotlib=use_matplotlib,
                fig_height=fig_height,
                fig_width=fig_width,
                polygon=self._wall_polygons[tag],
                alpha=alpha,
                line_width=line_width,
                line_style=line_style)
        if plot_interior:
            for tag in self._interior_polygons:
                fig, ax = plot_shapely_geometry(
                    fig=fig,
                    ax=ax,
                    color=interior_geo_color,
                    use_matplotlib=use_matplotlib,
                    fig_height=fig_height,
                    fig_width=fig_width,
                    polygon=self._interior_polygons[tag],
                    alpha=0.3,
                    line_width=line_width,
                    line_style=line_style)

        if plot_reference:
            fig, ax = self.plot_reference_geometries(
                fig=fig,
                ax=ax,
                color=reference_geo_color,
                use_matplotlib=use_matplotlib,
                fig_height=fig_height,
                fig_width=fig_width,
                line_style='dashed',
                alpha=0.5,
                line_width=3)

        return fig, ax

    def plot_reference_geometries(
            self,
            fig=None,
            ax=None,
            fig_width=20,
            fig_height=15,
            use_matplotlib=True,
            add_floorplan_reference_geometry_geometries=True,
            alpha=0.5,
            line_width=2,
            line_style='solid',
            color='tab:blue'):
        for geo in self._geometries:
            fig, ax = plot_shapely_geometry(
                fig=fig,
                ax=ax,
                color=color,
                use_matplotlib=use_matplotlib,
                fig_height=fig_height,
                fig_width=fig_width,
                polygon=geo,
                alpha=alpha,
                line_width=line_width,
                line_style=line_style)
        return fig, ax
