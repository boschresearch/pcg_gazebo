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
from shapely.geometry import MultiPoint, Polygon, MultiPolygon, \
    LineString, MultiLineString
from shapely.ops import triangulate, unary_union, polygonize
from time import time
from ..log import PCG_ROOT_LOGGER
from ..visualization import create_scene
from ..simulation import SimulationModel, ModelGroup


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
        step_x,
        step_y,
        z_levels=None,
        x_limits=None,
        y_limits=None,
        z_limits=None,
        model_name=None,
        mesh_type='collision',
        is_ground_plane=False,
        method='slices'):
    start_time = time()
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
    PCG_ROOT_LOGGER.info(
        'Generating horizontal rays with step_x={},'
        ' step_y={}, z_levels={}, model={}'.format(
            step_x, step_y, z_levels, model_name))

    if np.abs(x_limits[1] - x_limits[0]) <= step_x:
        step_x = np.abs(x_limits[1] - x_limits[0]) / 10.0

    if np.abs(y_limits[1] - y_limits[0]) <= step_y:
        step_y = np.abs(y_limits[1] - y_limits[0]) / 10.0

    x_samples = np.arange(x_limits[0], x_limits[1] + step_x, step_x)
    y_samples = np.arange(y_limits[0], y_limits[1] + step_y, step_y)

    if z_levels is None:
        n_levels = 5 if method == 'slices' else 10
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

    if method == 'rays':
        # Generating ray origins on the LEFT scene
        ray_origins = None
        # FIXME: Optimize generation of ray origins for large z_levels vectors
        for z in z_levels:
            horz_origin = np.vstack((
                x_limits[0] * np.ones(y_samples.size),
                y_samples,
                z * np.ones(y_samples.size))).T

            if ray_origins is None:
                ray_origins = horz_origin
            else:
                ray_origins = np.vstack((ray_origins, horz_origin))

        ray_directions = np.array([[1, 0, 0]
                                   for _ in range(ray_origins.shape[0])])

        # Generating ray origins on the BOTTOM of scene
        for z in z_levels:
            vert_origin = np.vstack((
                x_samples,
                y_limits[0] * np.ones(x_samples.size),
                z * np.ones(x_samples.size))).T

            ray_origins = np.vstack((ray_origins, vert_origin))
            ray_directions = np.vstack((ray_directions, np.array(
                [[0, 1, 0] for _ in range(x_samples.size)])))

        occupied_areas = list()
        PCG_ROOT_LOGGER.info(
            'Computing the intersections from horizontal '
            'rays, model={}, # rays={}'.format(
                model_name, ray_origins.shape[0]))

        ray_intersections = None

        meshes = model.get_meshes(mesh_type)
        for mesh in meshes:
            locations, index_ray, index_tri = mesh.ray.intersects_location(
                ray_origins=ray_origins,
                ray_directions=ray_directions)

            if len(locations) == 0:
                continue

            locations = locations[:, 0:2]
            locations = np.unique(locations, axis=0)

            if ray_intersections is None:
                ray_intersections = locations
            else:
                ray_intersections = np.vstack((ray_intersections, locations))

            points = MultiPoint(locations)
            # Dilate points and add them to the occupied area
            occupied_areas.append(points.buffer(np.max([step_x, step_y])))

        if not is_ground_plane:
            def _find_footprint_with_rays(xvec, yvec):
                x, y = np.meshgrid(x_samples, y_samples)
                PCG_ROOT_LOGGER.info(
                    'Finding footprint with vertical'
                    ' rays, model={}, # rays={}'.format(
                        model_name, x.size))
                footprint_areas = list()

                ray_origins = np.vstack((
                    x.flatten(),
                    y.flatten(),
                    np.max(z_levels) * np.ones(x.size))).T
                ray_directions = np.array(
                    [[0, 0, -1] for _ in range(ray_origins.shape[0])])

                for mesh in meshes:
                    locations, index_ray, index_tri = \
                        mesh.ray.intersects_location(
                            ray_origins=ray_origins,
                            ray_directions=ray_directions,
                            multiple_hits=False)

                    if len(locations) == 0:
                        continue

                    locations = locations[:, 0:2]
                    locations = np.unique(locations, axis=0)
                    locations = np.vstack((ray_intersections, locations))

                    points = MultiPoint(locations)
                    # Dilate points and add them to the occupied area
                    footprint_areas.append(
                        points.buffer(np.max([step_x, step_y])))
                PCG_ROOT_LOGGER.info(
                    'Vertical ray tracing done, model={}'.format(model_name))
                return footprint_areas

            x_samples = np.arange(x_limits[0], x_limits[1] + step_x, step_x)
            y_samples = np.arange(y_limits[0], y_limits[1] + step_y, step_y)

            if x_samples.shape[0] * y_samples.shape[0] > 1e5:
                try:
                    # Extract only the unique point intersections
                    ray_intersections = np.unique(ray_intersections, axis=0)
                    # Generating ray origins from the maximum Z level limit
                    PCG_ROOT_LOGGER.info(
                        'Performing triangulation, model={}'.format(
                            model_name))
                    triangles = triangulate(MultiPoint(ray_intersections))
                    PCG_ROOT_LOGGER.info('# triangles={}, model={}'.format(
                        len(triangles), model_name))

                    # Filter out the triangles that belong to the
                    # mesh footprint
                    max_z_level = np.max(z_levels)
                    ray_origins = np.array(
                        [[
                            t.centroid.xy[0][0],
                            t.centroid.xy[1][0],
                            max_z_level] for t in triangles])
                    ray_directions = np.array(
                        [[0, 0, -1] for _ in range(ray_origins.shape[0])])

                    idx = None
                    PCG_ROOT_LOGGER.info(
                        'Checking triangles centroids for'
                        ' intersections with mesh, model={}'.format(
                            model_name))
                    for mesh in meshes:
                        locations, index_ray, index_tri = \
                            mesh.ray.intersects_location(
                                ray_origins=ray_origins,
                                ray_directions=ray_directions,
                                multiple_hits=False)

                        if idx is None:
                            idx = np.array(index_ray)
                        else:
                            idx = np.unique(
                                np.hstack((idx, np.array(index_ray))))

                    PCG_ROOT_LOGGER.info(
                        'Triangles filtered for intersections,'
                        ' model={}, # triangles={}'.format(
                            model_name, idx.shape[0]))
                    occupied_areas = occupied_areas + \
                        [triangles[i] for i in idx]
                    PCG_ROOT_LOGGER.info(
                        'Finished triangulation, model={}'.format(model_name))
                except ValueError as ex:
                    PCG_ROOT_LOGGER.warning(
                        'triangulation failed, using vertical rays'
                        ' instead, model={}, message={}'.format(
                            model_name, ex))
                    occupied_areas = occupied_areas + \
                        _find_footprint_with_rays(x_samples, y_samples)
            else:
                PCG_ROOT_LOGGER.info(
                    'Applying vertical rays to find'
                    ' footprint, model={}'.format(model_name))
                occupied_areas = occupied_areas + \
                    _find_footprint_with_rays(x_samples, y_samples)
        else:
            PCG_ROOT_LOGGER.info(
                'Using only horizontal rays, model={}'.format(model_name))

        # Combine all occupied areas
        occupied_areas = unary_union(occupied_areas)

        # Dilate and erode the polygon to get rid of small gaps
        occupied_areas = occupied_areas.buffer(
            np.max([step_x, step_y]) / 2).buffer(-np.max([step_x, step_y]) / 2)

        # Remove interior polygons, only if model is not a ground plane model
        if not is_ground_plane:
            if isinstance(occupied_areas, Polygon):
                for interior_poly in occupied_areas.interiors:
                    interior_poly = Polygon(interior_poly)
                    occupied_areas = occupied_areas.union(interior_poly)
            elif isinstance(occupied_areas, MultiPolygon):
                for geo in occupied_areas.geoms:
                    for interior_poly in geo.interiors:
                        interior_poly = Polygon(interior_poly)
                        geo = geo.union(interior_poly)

        occupied_areas = occupied_areas.simplify(
            0.001, preserve_topology=False)

        PCG_ROOT_LOGGER.info(
            'Footprint of sliced model ready, model={}, total time={}'.format(
                model_name, time() - start_time))
    elif method == 'slices':
        plane_normal = [0, 0, 1]

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
            lines = list()
            for section in sections:
                for poly in section.entities:
                    line = LineString(
                        section.vertices[poly.points])
                    lines.append(line)
            footprint_lines = unary_union(MultiLineString(lines))
            geoms = MultiPolygon(list(polygonize(footprint_lines)))

            for geo in geoms:
                if is_ground_plane:
                    filtered_geoms.append(geo)
                else:
                    for z in z_levels:
                        if z <= model_z_limits[0]:
                            z = model_z_limits[0] + 0.01
                        elif z >= model_z_limits[1]:
                            z = model_z_limits[1] - 0.01
                        ray_directions = list()
                        ray_origins = list()
                        for theta in np.linspace(0, np.pi, 10):
                            ray_directions.append(
                                [np.cos(theta), np.sin(theta), 0])
                            ray_origins.append(
                                [
                                    geo.centroid.xy[0][0],
                                    geo.centroid.xy[1][0],
                                    z
                                ]
                            )

                        for d, o in zip(ray_directions, ray_origins):
                            locations = mesh.ray.intersects_location(
                                ray_origins=[o],
                                ray_directions=[d])
                            if len(locations[0]) % 2 != 0:
                                inside_mesh = True
                                break
                        inside_mesh = len(locations[0]) % 2 != 0
                    if inside_mesh:
                        filtered_geoms.append(geo)
        occupied_areas = unary_union(MultiPolygon(filtered_geoms))
    else:
        raise ValueError('Wrong method for occupancy grid computation')
    return occupied_areas


def _get_occupied_area_proc(args):
    from ..simulation import SimulationModel
    model = SimulationModel.from_sdf(args[0])

    occupied_areas = get_occupied_area(
        model,
        args[1],
        args[2],
        args[3],
        x_limits=None,
        y_limits=None,
        z_limits=None,
        model_name=args[4],
        mesh_type=args[5],
        is_ground_plane=args[6],
        method=args[7])
    return occupied_areas


def generate_occupancy_grid(
        models,
        z_levels=None,
        x_limits=None,
        y_limits=None,
        z_limits=None,
        step_x=0.1,
        step_y=0.1,
        n_processes=10,
        mesh_type='collision',
        ground_plane_models=None,
        method='slices'):
    if len(models) == 0:
        PCG_ROOT_LOGGER.warning(
            'List of models is empty, cannot compute occupancy grid')
        return None

    if ground_plane_models is None:
        ground_plane_models = list()

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
        ground_plane=dict())

    pool = Pool(n_processes)

    if len(models):
        non_gp_models = list()
        for tag in models:
            if models[tag].is_ground_plane or \
                    tag in ground_plane_models:
                continue
            non_gp_models.append(
                [
                    models[tag].to_sdf(),
                    step_x,
                    step_y,
                    z_levels,
                    tag,
                    mesh_type,
                    False,
                    method
                ]
            )

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
            if models[model_name].is_ground_plane or \
                    model_name in ground_plane_models:
                continue
            elif models[model_name].static:
                PCG_ROOT_LOGGER.info(
                    'Adding static model occupied space,'
                    ' model={}, area={}'.format(
                        model_name, model_occupied_area.area))
                occupancy_output['static'][model_name] = model_occupied_area
            else:
                PCG_ROOT_LOGGER.info(
                    'Adding non-static model occupied space,'
                    ' model={}, area={}'.format(
                        model_name, model_occupied_area.area))
                occupancy_output['non_static'][model_name] = \
                    model_occupied_area
    else:
        PCG_ROOT_LOGGER.info('No non-ground-plane models available')

    # Compute the occupancy map for the ground plane models, if any exist
    for tag in models:
        if not models[tag].is_ground_plane and tag not in ground_plane_models:
            continue
        model_occupied_area = get_occupied_area(
            models[tag],
            step_x,
            step_y,
            z_levels,
            mesh_type=mesh_type,
            is_ground_plane=False,
            method=method)
        occupancy_output['static'][tag] = model_occupied_area

        model_occupied_area = get_occupied_area(
            models[tag],
            step_x,
            step_y,
            z_levels,
            mesh_type=mesh_type,
            is_ground_plane=True,
            method=method)
        occupancy_output['ground_plane'][tag] = model_occupied_area

    PCG_ROOT_LOGGER.info('Computation of occupancy grid finished')
    return occupancy_output
