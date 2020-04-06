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
from multiprocessing.pool import Pool
from shapely.geometry import Polygon, MultiPolygon, \
    LineString
from shapely.ops import unary_union, polygonize, linemerge
from ..log import PCG_ROOT_LOGGER
from ..visualization import create_scene
from ..simulation import SimulationModel, ModelGroup
from ..utils import has_string_pattern


def _get_model_limits(model, mesh_type='collision'):
    x_limits = None
    y_limits = None
    z_limits = None

    meshes = list()
    if isinstance(model, SimulationModel) or isinstance(model, ModelGroup):
        PCG_ROOT_LOGGER.info(
            'Processing the bounds of simulation model={}'.format(
                model.name))
        meshes = model.get_meshes(mesh_type)
    elif isinstance(model, trimesh.Trimesh):
        meshes = [model]
    else:
        msg = 'Input is neither of SimulationModel' \
            ' or Trimesh type, provided={}'.format()
        PCG_ROOT_LOGGER.error(msg)
        raise ValueError(msg)

    for mesh in meshes:
        bounds = mesh.bounds
        if x_limits is None:
            x_limits = bounds[:, 0].flatten()
        else:
            x_limits[0] = min(x_limits[0], bounds[0, 0])
            x_limits[1] = max(x_limits[1], bounds[1, 0])

        if y_limits is None:
            y_limits = bounds[:, 1].flatten()
        else:
            y_limits[0] = min(y_limits[0], bounds[0, 1])
            y_limits[1] = max(y_limits[1], bounds[1, 1])

        if z_limits is None:
            z_limits = bounds[:, 2].flatten()
        else:
            z_limits[0] = min(z_limits[0], bounds[0, 2])
            z_limits[1] = max(z_limits[1], bounds[1, 2])
    return x_limits, y_limits, z_limits


def get_occupied_area(
        model,
        z_levels=None,
        x_limits=None,
        y_limits=None,
        z_limits=None,
        model_name=None,
        mesh_type='collision',
        is_ground_plane=False):
    PCG_ROOT_LOGGER.info('get_occupied_area(), model={}'.format(model_name))
    PCG_ROOT_LOGGER.info('get_occupied_area(), mesh_type={}'.format(mesh_type))

    model_x_limits, model_y_limits, model_z_limits = _get_model_limits(model)
    PCG_ROOT_LOGGER.info(
        'Model limits calculated, model={}, x_limits={},'
        ' y_limits={}, z_limits={}'.format(
            model_name,
            model_x_limits,
            model_y_limits,
            model_z_limits))

    if x_limits is None:
        x_limits = model_x_limits
    else:
        x_limits = np.array(x_limits).flatten()
        if x_limits.size != 2:
            PCG_ROOT_LOGGER.error(
                'Input x_limits must have two elements, provided={}'.format(
                    x_limits.size))
            return None
        if x_limits[0] >= x_limits[1]:
            PCG_ROOT_LOGGER.error(
                'Input x_limits[0] must be smaller than'
                ' x_limits[1], provided={}'.format(z_limits))
            return None

    if y_limits is None:
        y_limits = model_y_limits
    else:
        y_limits = np.array(y_limits).flatten()
        if y_limits.size != 2:
            PCG_ROOT_LOGGER.error(
                'Input y_limits must have two elements,'
                ' provided={}'.format(
                    x_limits.size))
            return None
        if y_limits[0] >= y_limits[1]:
            PCG_ROOT_LOGGER.error(
                'Input y_limits[0] must be smaller than'
                ' y_limits[1], provided={}'.format(z_limits))
            return None

    if z_limits is None:
        z_limits = model_z_limits
    else:
        z_limits = np.array(z_limits).flatten()
        if z_limits.size != 2:
            PCG_ROOT_LOGGER.error(
                'Input z_limits must have two elements, provided={}'.format(
                    z_limits.size))
            return None
        if z_limits[0] >= z_limits[1]:
            PCG_ROOT_LOGGER.error(
                'Input z_limits[0] must be smaller than'
                ' z_limits[1], provided={}'.format(z_limits))
            return None

    PCG_ROOT_LOGGER.info(
        'Computing occupied area, model={}, x_limits={},'
        ' y_limits={}, z_limits={}'.format(
            model_name, x_limits, y_limits, z_limits))
    step_x = 0.01
    step_y = 0.01

    if z_levels is None:
        n_levels = 10
        z_levels = np.linspace(z_limits[0], z_limits[1], n_levels)

    if is_ground_plane:
        z_levels = [z_limits[0]] + z_levels

    z_levels = np.array(z_levels)
    n_levels = z_levels.size
    min_z_level, max_z_level = np.min(z_levels), np.max(z_levels)
    z_levels = z_levels[np.nonzero(np.logical_and(
        z_levels >= model_z_limits[0], z_levels <= model_z_limits[1]))[0]]

    if z_levels.size == 0:
        PCG_ROOT_LOGGER.warning(
            'All Z height levels were outside'
            ' of the model height range!, model={}, '
            'model Z limits={}, Z level limits={}'.format(
                model_name, model_z_limits, (min_z_level, max_z_level)))
        return None
    PCG_ROOT_LOGGER.info(
        'Filtering out Z height levels outside of the model '
        'height range, model={}, # levels before={}, # levels'
        ' after={}'.format(model_name, n_levels, z_levels.size))

    def _is_interior_polygon(current_mesh, current_geo):
        if not isinstance(current_geo, (Polygon, MultiPolygon)):
            return False
        if is_ground_plane:
            return True
        else:
            inside_mesh = False
            for z in z_levels:
                if z <= model_z_limits[0]:
                    z = model_z_limits[0] + 0.01
                elif z >= model_z_limits[1]:
                    z = model_z_limits[1] - 0.01
                ray_directions = list()
                ray_origins = list()

                point = current_geo.representative_point()

                ray_directions = [
                    [0, 0, 1],
                    [0, 0, -1]
                ]
                ray_origins = \
                    [
                        [point.xy[0][0], point.xy[1][0], z]
                        for _ in range(len(ray_directions))
                    ]

                locations = current_mesh.ray.intersects_location(
                    ray_origins=ray_origins,
                    ray_directions=ray_directions)

                if len(locations[1]) == 0:
                    return False

                n_theta = 15
                for theta in np.linspace(0, np.pi, n_theta):
                    ray_directions.append(
                        [np.cos(theta), np.sin(theta), 0])
                    ray_origins.append(
                        [point.xy[0][0], point.xy[1][0], z]
                    )

                locations = current_mesh.ray.intersects_location(
                    ray_origins=ray_origins,
                    ray_directions=ray_directions)

                unique, counts = np.unique(
                    locations[1], return_counts=True)
                for c in counts:
                    if c % 2 != 0:
                        inside_mesh = True
                        break

                if inside_mesh:
                    break

            return inside_mesh

    plane_normal = [0, 0, 1]
    occupied_areas = list()
    meshes = model.get_meshes(mesh_type)
    filtered_geoms = list()

    for mesh in meshes:
        if model_z_limits[0] in z_levels:
            z_levels = np.delete(
                z_levels,
                np.where(z_levels == model_z_limits[0]))
        if model_z_limits[1] in z_levels:
            z_levels = np.delete(
                z_levels,
                np.where(z_levels == model_z_limits[0]))
        if z_levels.size == 0:
            z_levels = np.array(
                [(model_z_limits[1] - model_z_limits[0]) / 2 +
                    model_z_limits[0]])

        sections = mesh.section_multiplane(
            plane_origin=[0, 0, 0],
            plane_normal=plane_normal,
            heights=z_levels)
        sections = [s for s in sections if s is not None]
        polys = list()
        for section in sections:
            lines = list()
            for poly in section.entities:
                line = LineString(
                    section.vertices[poly.points])
                lines.append(line)
            boundaries = linemerge(lines)

            polygons_from_boundaries = MultiPolygon(polygonize(boundaries))

            if mesh.is_watertight and polygons_from_boundaries.area > 0:
                polys = polys + list(polygonize(boundaries))
            else:
                p = boundaries.envelope.difference(boundaries.buffer(1e-3))
                for geo in p.geoms:
                    polys.append(geo.buffer(1e-3))

        polys = MultiPolygon(polys)
        if not is_ground_plane:
            for i in range(len(polys.geoms)):
                if _is_interior_polygon(mesh, polys.geoms[i]):
                    filtered_geoms.append(polys.geoms[i])
            occupied_areas.append(MultiPolygon(filtered_geoms))
        else:
            if polys.area > 0:
                occupied_areas.append(polys)

    if len(occupied_areas) > 1:
        occupied_areas = unary_union(
            [geo.buffer(max(step_x, step_y) / 2) for geo in occupied_areas])
        occupied_areas = occupied_areas.buffer(-max(step_x, step_y) / 2)
    else:
        occupied_areas = unary_union(occupied_areas[0])

    if is_ground_plane:
        dilated_footprint = occupied_areas.buffer(max(step_x, step_y))
        full_footprint = MultiPolygon(
            list(polygonize(dilated_footprint.boundary)))
        occupied_areas = occupied_areas.union(full_footprint)
        occupied_areas = occupied_areas.buffer(-max(step_x, step_y))
    if occupied_areas.is_empty:
        PCG_ROOT_LOGGER.warning(
            'Footprint for model {} could not be '
            'computed for z_levels={}, model Z limits={}'.format(
                model.name, z_levels, model_z_limits))
        return None
    # TODO: Set x_limits and y_limits to the occupied area
    return occupied_areas


def _get_occupied_area_proc(args):
    from ..simulation import SimulationModel
    model = SimulationModel.from_sdf(args[0])

    occupied_areas = get_occupied_area(
        model,
        z_levels=args[1],
        x_limits=None,
        y_limits=None,
        z_limits=None,
        model_name=args[2],
        mesh_type=args[3],
        is_ground_plane=args[4])
    return occupied_areas


def generate_occupancy_grid(
        models,
        z_levels=None,
        x_limits=None,
        y_limits=None,
        z_limits=None,
        n_processes=None,
        mesh_type='collision',
        ground_plane_models=None):
    if len(models) == 0:
        PCG_ROOT_LOGGER.warning(
            'List of models is empty, cannot compute occupancy grid')
        return None

    if ground_plane_models is None:
        ground_plane_models = list()

    def _is_ground_plane(model):
        if model.is_ground_plane:
            return True
        for tag in ground_plane_models:
            if has_string_pattern(model.name, tag):
                return True
        return False

    scene = create_scene(list(models.values()))

    if x_limits is None:
        x_limits = scene.bounds[:, 0].flatten()
    else:
        x_limits = np.array(x_limits).flatten()
        if x_limits.size != 2:
            PCG_ROOT_LOGGER.error(
                'Input x_limits must have two elements, provided={}'.format(
                    x_limits.size))
            return None
        if x_limits[0] >= x_limits[1]:
            PCG_ROOT_LOGGER.error(
                'Input x_limits[0] must be smaller than'
                ' x_limits[1], provided={}'.format(z_limits))
            return None

    if y_limits is None:
        y_limits = scene.bounds[:, 1].flatten()
    else:
        y_limits = np.array(y_limits).flatten()
        if y_limits.size != 2:
            PCG_ROOT_LOGGER.error(
                'Input y_limits must have two elements, provided={}'.format(
                    x_limits.size))
            return None
        if y_limits[0] >= y_limits[1]:
            PCG_ROOT_LOGGER.error(
                'Input y_limits[0] must be smaller than'
                ' y_limits[1], provided={}'.format(z_limits))
            return None

    if z_limits is None:
        z_limits = scene.bounds[:, 2].flatten()
    else:
        z_limits = np.array(z_limits).flatten()
        if z_limits.size != 2:
            PCG_ROOT_LOGGER.error(
                'Input z_limits must have two elements, provided={}'.format(
                    z_limits.size))
            return None
        if z_limits[0] >= z_limits[1]:
            PCG_ROOT_LOGGER.error(
                'Input z_limits[0] must be smaller than'
                ' z_limits[1], provided={}'.format(z_limits))
            return None

    PCG_ROOT_LOGGER.info(
        'Generating occupancy grid in intervals, x_limits={},'
        'y_limits={}, z_limits={}'.format(
            x_limits, y_limits, z_limits))
    PCG_ROOT_LOGGER.info('Generating grid using {} meshes'.format(mesh_type))

    model_occupied_areas = list()

    PCG_ROOT_LOGGER.info('List of models={}'.format(list(models.keys())))

    occupancy_output = dict(
        static=dict(),
        non_static=dict(),
        ground_plane=None)

    pool = Pool(n_processes)

    if len(models):
        non_gp_models = list()
        for tag in models:
            if _is_ground_plane(models[tag]):
                continue
            non_gp_models.append(
                [
                    models[tag].to_sdf(),
                    z_levels,
                    tag,
                    mesh_type,
                    False
                ]
            )

        if len(non_gp_models):
            results = pool.map(
                _get_occupied_area_proc,
                non_gp_models)

            for model_occupied_area, model_name in zip(
                    results, [x[0].name for x in non_gp_models]):
                if model_occupied_area is None:
                    PCG_ROOT_LOGGER.warning(
                        'No footprint found for model {}'
                        ' for the given parameters'.format(model_name))
                    continue
                model_occupied_areas.append(model_occupied_area)

                if _is_ground_plane(models[model_name]):
                    continue
                elif models[model_name].static:
                    PCG_ROOT_LOGGER.info(
                        'Adding static model occupied space,'
                        ' model={}, area={}'.format(
                            model_name, model_occupied_area.area))
                    occupancy_output['static'][model_name] = \
                        model_occupied_area
                else:
                    PCG_ROOT_LOGGER.info(
                        'Adding non-static model occupied space,'
                        ' model={}, area={}'.format(
                            model_name, model_occupied_area.area))
                    occupancy_output['non_static'][model_name] = \
                        model_occupied_area
    else:
        PCG_ROOT_LOGGER.info('No non-ground-plane models available')

    # Compute the ground plane region from the model group formed by all
    # the models flagged as part of the ground plane
    ground_plane_group = ModelGroup()

    # Compute the occupancy map for the ground plane models, if any exist
    for tag in models:
        if not _is_ground_plane(models[tag]):
            continue
        # Add model to ground plane model group
        ground_plane_group.add_model(tag, models[tag])

    if ground_plane_group.n_models > 0:
        occupancy_output['static']['ground_plane_models'] = get_occupied_area(
            ground_plane_group,
            z_levels,
            mesh_type=mesh_type,
            is_ground_plane=False)

        if ground_plane_group.n_models > 0:
            model_occupied_area = get_occupied_area(
                ground_plane_group,
                z_levels,
                mesh_type=mesh_type,
                is_ground_plane=True)
            # Check if the places occupied by the ground plane models
            # are equal to the computed free space
            diff = model_occupied_area.difference(
                occupancy_output['static']['ground_plane_models'])

            if model_occupied_area.almost_equals(
                    occupancy_output['static']['ground_plane_models'],
                    decimal=3) or diff.area < 1e-3:
                print(
                    'The areas for free space are equivalent to the '
                    'occupied areas by the ground plane occupied '
                    'areas. Setting limited ground plane to None')
                occupancy_output['ground_plane'] = None
            else:
                occupancy_output['ground_plane'] = model_occupied_area

    PCG_ROOT_LOGGER.info('Computation of occupancy grid finished')
    return occupancy_output
