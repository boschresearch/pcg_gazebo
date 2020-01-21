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
import trimesh
from trimesh.creation import extrude_polygon
from shapely.geometry import Polygon, Point, \
    MultiPoint, MultiPolygon
from shapely.ops import unary_union
from ..log import PCG_ROOT_LOGGER


def extrude(polygon, height, thickness=0, cap_style='round',
            join_style='round', extrude_boundaries=False):
    assert height > 0, 'Extrude height must be greater than zero'
    if isinstance(polygon, Polygon) and not extrude_boundaries:
        assert polygon.is_valid, 'The input polygon object' \
            ' is an invalid polygon shape'
        PCG_ROOT_LOGGER.info(
            'Extruding a polygon to generate the model\'s mesh')
        processed_polygon = polygon
    elif isinstance(polygon, MultiPolygon) and not extrude_boundaries:
        processed_polygon = unary_union(polygon)
    else:
        assert thickness > 0, 'For any point or line-like input geometries, ' \
            'a thickness has to be provided to generate the polygon by' \
            ' dilating the input object'
        cs_indexes = ['round', 'flat', 'square']
        assert cap_style in cs_indexes, \
            'Invalid cap style to dilate the geometry'
        js_indexes = ['round', 'mitre', 'bevel']
        assert join_style in js_indexes, \
            'Invalid join style to dilate the geometry'

        # In case the provided polygon are of type Polygon or MultiPolygon,
        # their boundaries will be extracted
        if isinstance(polygon, Polygon) or isinstance(polygon, MultiPolygon):
            input_poly = unary_union(polygon.boundary)
        else:
            input_poly = polygon

        PCG_ROOT_LOGGER.info(
            'Dilating the input geometry, thickness={}, cap_style={}, '
            'join_style={}'.format(thickness, cap_style, join_style))
        processed_polygon = input_poly.buffer(
            thickness,
            cap_style=cs_indexes.index(cap_style) + 1,
            join_style=js_indexes.index(join_style) + 1)

        if isinstance(processed_polygon, MultiPolygon):
            processed_polygon = unary_union(processed_polygon)
        if not hasattr(processed_polygon, 'exterior'):
            PCG_ROOT_LOGGER.warning(
                'After dilating input polygon, the resulting polygon has no'
                ' \'exterior\' attribute, using the convex hull instead')
            processed_polygon = polygon.convex_hull

    return extrude_polygon(processed_polygon, height)


def sweep():
    raise NotImplementedError()


def box(size):
    import collections
    assert isinstance(size, collections.Iterable), 'Size is not an array'
    vec = list(size)
    assert len(vec) == 3, 'Input size array must have 3 elements'
    for elem in vec:
        assert elem > 0, 'Size vector components must be greater than zero'
    return trimesh.creation.box(extents=size)


def capsule(radius=1, height=1):
    assert radius > 0, 'Capsule radius must be greater than zero'
    assert height > 0, 'Capsule height must be greater than zero'
    return trimesh.creation.capsule(radius=radius, height=height)


def cylinder(radius, height):
    assert radius > 0, 'Cylinder radius must be greater than zero'
    assert height > 0, 'Cylinder height must be greater than zero'
    return trimesh.creation.cylinder(radius=radius, height=height)


def sphere(radius):
    assert radius > 0, 'Sphere radius must be greater than zero'
    return trimesh.creation.icosphere(radius=radius)


def room(polygon, wall_height=2, wall_thickness=0.1, single_mesh=False,
         add_floor=True, add_ceiling=True, floor_thickness=0.01):
    assert not isinstance(polygon, Point) and \
        not isinstance(polygon, MultiPoint), \
        'A room cannot be created from a point or multiple points'
    assert not isinstance(
        polygon, MultiPolygon), 'A room cannot be created' \
        ' from a MultiPolygon object'

    # Build walls
    wall_mesh = extrude(
        polygon,
        height=wall_height,
        thickness=wall_thickness,
        cap_style='square',
        join_style='mitre',
        extrude_boundaries=True)

    return wall_mesh
