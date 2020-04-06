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
from .log import PCG_ROOT_LOGGER
try:
    from bokeh.plotting import figure
    import bokeh.palettes
    BOKEH_AVAILABLE = True
except ImportError as ex:
    PCG_ROOT_LOGGER.warning(ex)
    BOKEH_AVAILABLE = False
try:
    from matplotlib import pyplot as plt
    from matplotlib import cm
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import descartes
    MATPLOTLIB_AVAILABLE = True
except ImportError as ex:
    PCG_ROOT_LOGGER.warning(ex)
    MATPLOTLIB_AVAILABLE = False
import os
import numpy as np
import trimesh
from shapely.geometry import Polygon, MultiPolygon, \
    LineString, MultiLineString, Point, MultiPoint
from multiprocessing.pool import ThreadPool


def _get_footprints(inputs):
    fp = inputs[0].get_footprint(mesh_type=inputs[1], z_limits=inputs[2])
    return fp


def get_axes(fig=None, engine='matplotlib', fig_width=20, fig_height=15,
             projection=None):
    if fig is None:
        fig = get_figure(engine=engine, fig_width=fig_width,
                         fig_height=fig_height)
    if fig_height is None:
        fig_height = 10 if engine == 'matplotlib' else 400

    if fig_width is None:
        fig_width = 15 if engine == 'matplotlib' else 800

    if engine == 'matplotlib':
        ax = fig.add_subplot(111, projection=projection)
    else:
        ax = None
    return fig, ax


def get_figure(engine='matplotlib', fig_width=20, fig_height=15):
    if engine == 'matplotlib':
        assert MATPLOTLIB_AVAILABLE, 'matplotlib is not available!'
    elif engine == 'bokeh':
        assert BOKEH_AVAILABLE, 'bokeh is not available!'
    else:
        raise ValueError(
            'Plotting engine <{}> is not available'.format(engine))

    use_matplotlib = engine == 'matplotlib'
    if not BOKEH_AVAILABLE:
        use_matplotlib = True

    if fig_height is None:
        fig_height = 10 if engine == 'matplotlib' else 400

    if fig_width is None:
        fig_width = 15 if engine == 'matplotlib' else 800

    if use_matplotlib:
        fig = plt.figure(figsize=(fig_width, fig_height))
    else:
        fig = figure(plot_width=fig_width, plot_height=fig_height)
    return fig


def plot_mesh(
        mesh,
        fig=None,
        ax=None,
        alpha=0.5,
        line_width=2,
        color=None,
        line_style='solid',
        fig_width=20,
        fig_height=15,
        marker_style='o',
        grid=True):
    assert MATPLOTLIB_AVAILABLE, \
        'To plot mesh edges, matplotlib must be available'
    if ax is None:
        fig, ax = get_axes(
            fig=fig,
            engine='matplotlib',
            fig_width=fig_width,
            fig_height=fig_height,
            projection='3d')

    pc = Poly3DCollection(
        [[mesh.vertices[i] for i in face] for face in mesh.faces])
    pc.set_alpha(alpha)
    pc.set_facecolor(color)
    pc.set_edgecolor(color)

    edges = mesh.vertices[mesh.edges]
    ax.add_collection3d(pc)
    ax.plot(
        edges[:, :, 0].flatten(),
        edges[:, :, 1].flatten(),
        edges[:, :, 2].flatten(),
        color=color,
        alpha=alpha,
        linewidth=line_width,
        zorder=2,
        marker=marker_style,
        linestyle=line_style
    )

    ax.grid(grid)
    return fig, ax


def plot_shapely_geometry(
        polygon=None,
        fig=None,
        ax=None,
        alpha=0.5,
        line_width=2,
        legend=None,
        color=None,
        line_style='solid',
        use_matplotlib=True,
        fig_width=20,
        fig_height=15,
        marker_style='o',
        grid=True):

    if not use_matplotlib:
        assert BOKEH_AVAILABLE, 'Bokeh is not available!'
        if fig is None:
            fig = get_figure(
                engine='bokeh',
                fig_width=fig_width,
                fig_height=fig_height)
        if isinstance(polygon, (Polygon, MultiPolygon)):
            vertices = np.concatenate(
                [np.asarray(polygon.exterior)[:, :2]] +
                [np.asarray(r)[:, :2] for r in polygon.interiors])
            fig.patch(
                vertices[:, 0],
                vertices[:, 1],
                alpha=alpha,
                line_width=line_width,
                legend_label=legend,
                color=color,
                line_dash=line_style)
    else:
        if ax is None:
            fig, ax = get_axes(
                fig=fig,
                engine='matplotlib',
                fig_width=fig_width,
                fig_height=fig_height)

        if isinstance(polygon, Polygon):
            patch = descartes.PolygonPatch(
                polygon,
                facecolor=color,
                edgecolor='black',
                alpha=alpha,
                zorder=2,
                linestyle=line_style,
                label=legend)
            ax.add_patch(patch)
        elif isinstance(polygon, MultiPolygon):
            colors = cm.get_cmap(
                'viridis')(np.linspace(0, 1, len(polygon.geoms)))
            for geo, color in zip(polygon.geoms, colors):
                fig, ax = plot_shapely_geometry(
                    fig=fig,
                    ax=ax,
                    polygon=geo,
                    alpha=alpha,
                    line_width=line_width,
                    legend=legend,
                    color=color,
                    line_style=line_style,
                    use_matplotlib=use_matplotlib)
        elif isinstance(polygon, LineString):
            # Plot coordinates
            x, y = polygon.xy
            ax.plot(x, y, marker=marker_style, color=color, zorder=2)
            # Plot lines
            ax.plot(
                x,
                y,
                color=color,
                alpha=alpha,
                linewidth=line_width,
                zorder=2)
        elif isinstance(polygon, MultiLineString):
            colors = cm.get_cmap(
                'viridis')(np.linspace(0, 1, len(polygon.geoms)))
            for geo, color in zip(polygon.geoms, colors):
                fig, ax = plot_shapely_geometry(
                    fig=fig,
                    ax=ax,
                    polygon=geo,
                    alpha=alpha,
                    line_width=line_width,
                    marker_style=marker_style,
                    legend=legend,
                    color=color,
                    line_style=line_style,
                    use_matplotlib=use_matplotlib)
        elif isinstance(polygon, Point):
            # Plot coordinates
            x, y = polygon.xy
            ax.plot(x, y, marker=marker_style, color=color, zorder=2)
        elif isinstance(polygon, MultiPoint):
            for point in polygon.geoms:
                fig, ax = plot_shapely_geometry(
                    fig=fig,
                    ax=ax,
                    polygon=point,
                    alpha=alpha,
                    line_width=line_width,
                    marker_style=marker_style,
                    legend=legend,
                    color=color,
                    use_matplotlib=use_matplotlib)
        ax.axis('equal')
        ax.grid(grid)
    return fig, ax


def plot_workspace(
        workspace,
        fig=None,
        ax=None,
        fig_width=None,
        fig_height=None,
        color=None,
        alpha=0.5,
        line_width=2,
        legend=None,
        line_style='dashed',
        engine='matplotlib'):
    assert BOKEH_AVAILABLE or MATPLOTLIB_AVAILABLE, \
        'None of the plotting libraries matplotlib or bokeh could be imported'
    use_matplotlib = engine == 'matplotlib'
    if not BOKEH_AVAILABLE:
        use_matplotlib = True

    if fig_height is None:
        fig_height = 10 if use_matplotlib else 400

    if fig_width is None:
        fig_width = 15 if use_matplotlib else 800

    geo = workspace.get_geometry()

    if isinstance(geo, trimesh.base.Trimesh):
        fig, ax = plot_mesh(
            mesh=geo,
            fig=fig,
            ax=ax,
            alpha=alpha,
            line_width=line_width,
            color=color,
            line_style=line_style,
            fig_width=fig_width,
            fig_height=fig_height,
            marker_style='o',
            grid=True)
    else:
        fig, ax = plot_shapely_geometry(
            polygon=geo,
            fig=fig,
            ax=ax,
            alpha=alpha,
            line_width=line_width,
            legend=legend if legend is not None else 'workspace',
            color=color,
            line_style=line_style,
            use_matplotlib=use_matplotlib)

    return fig, ax


def plot_workspaces(
        workspaces,
        fig=None,
        ax=None,
        fig_width=800,
        fig_height=400,
        alpha=1,
        line_width=2,
        line_style='dashed',
        engine='bokeh',
        colormap='viridis'):
    assert BOKEH_AVAILABLE or MATPLOTLIB_AVAILABLE, \
        'None of the plotting libraries matplotlib or bokeh could be imported'
    use_matplotlib = engine == 'matplotlib'
    if not BOKEH_AVAILABLE:
        use_matplotlib = True

    if use_matplotlib:
        if isinstance(colormap, str):
            colors = cm.get_cmap(colormap)(np.linspace(0, 1, len(workspaces)))
    else:
        if isinstance(colormap, str):
            colors = getattr(bokeh.palettes, colormap)(len(workspaces))

    for tag, color in zip(workspaces.keys(), colors):
        fig = plot_workspace(
            workspaces[tag],
            fig=fig,
            ax=ax,
            color=color,
            alpha=alpha,
            line_width=line_width,
            line_style=line_style,
            legend=tag,
            engine=engine)
    return fig


def plot_footprint(
        footprint,
        fig=None,
        ax=None,
        fig_width=800,
        fig_height=400,
        color=None,
        alpha=0.5,
        line_width=2,
        legend=None,
        line_style='solid',
        engine='bokeh'):
    assert BOKEH_AVAILABLE or MATPLOTLIB_AVAILABLE, \
        'None of the plotting libraries matplotlib or bokeh could be imported'
    use_matplotlib = engine == 'matplotlib'
    if not BOKEH_AVAILABLE:
        use_matplotlib = True

    if isinstance(footprint, dict):
        for tag in footprint:
            fig, ax = plot_footprint(
                footprint=footprint[tag],
                fig=fig,
                ax=ax,
                fig_width=fig_width,
                fig_height=fig_height,
                color=color,
                alpha=alpha,
                line_width=line_width,
                legend=None,
                line_style=line_style,
                engine=engine)
    else:
        footprints = list()
        if isinstance(footprint, Polygon):
            footprints.append(footprint)
        elif isinstance(footprint, MultiPolygon):
            footprints = [t for t in footprint.geoms]

        if len(footprints) > 0:
            for fp in footprints:
                fig, ax = plot_shapely_geometry(
                    fig=fig,
                    ax=ax,
                    polygon=fp,
                    alpha=alpha,
                    line_width=line_width,
                    legend=legend,
                    color=color,
                    line_style=line_style,
                    use_matplotlib=use_matplotlib)

    return fig, ax


def plot_footprints(
        models,
        fig=None,
        ax=None,
        mesh_type='visual',
        fig_width=800,
        fig_height=400,
        alpha=0,
        line_width=2,
        n_processes=4,
        line_style='solid',
        engine='bokeh',
        colormap='magma',
        grid=True,
        ignore_ground_plane=True,
        z_limits=None,
        dpi=200):
    assert n_processes > 0, 'Number of footprint calculation processes '\
        'must be greater than zero'
    assert BOKEH_AVAILABLE or MATPLOTLIB_AVAILABLE, \
        'None of the plotting libraries matplotlib or bokeh could be imported'
    use_matplotlib = engine == 'matplotlib'
    if not BOKEH_AVAILABLE:
        use_matplotlib = True

    pool = ThreadPool(n_processes)
    if ignore_ground_plane:
        footprints = pool.map(
            _get_footprints, [
                (models[tag], mesh_type, z_limits)
                for tag in models if not models[tag].is_ground_plane])
        model_names = [
            tag for tag in models if not models[tag].is_ground_plane]
    else:
        footprints = pool.map(
            _get_footprints,
            [(models[tag], mesh_type, z_limits) for tag in models])
        model_names = [tag for tag in models]

    if use_matplotlib:
        if isinstance(colormap, str):
            colors = cm.get_cmap(colormap)(np.linspace(0, 1, len(model_names)))
        else:
            colors = [colormap for _ in range(len(model_names))]
    else:
        if isinstance(colormap, str):
            colors = getattr(bokeh.palettes, colormap)(len(model_names))
        else:
            colors = [colormap for _ in range(len(model_names))]

    for fp, color, model_name in zip(footprints, colors, model_names):
        PCG_ROOT_LOGGER.info(
            'Plotting footprint from model <{}>, polygons={}'.format(
                model_name, fp))
        for tag in fp:
            fig, ax = plot_footprint(
                fp[tag],
                fig=fig,
                ax=ax,
                color=color,
                alpha=alpha,
                line_width=line_width,
                legend=model_name,
                line_style=line_style,
                engine=engine)

    if use_matplotlib:
        ax.axis('equal')
        ax.grid(grid)
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        ax.autoscale(enable=True, axis='both', tight=True)

        ax.scatter(
            [models[t].pose.x for t in model_names],
            [models[t].pose.y for t in model_names],
            marker='o')
    else:
        fig.circle_cross(
            [models[t].pose.x for t in model_names],
            [models[t].pose.y for t in model_names],
            fill_color=color,
            line_color='black')

        fig.legend.click_policy = 'hide'

    return fig, ax


def create_scene(
        entities,
        mesh_type='collision',
        add_pseudo_color=False,
        alpha=0.5,
        add_axis=True):
    scene = trimesh.scene.Scene()
    if add_axis:
        scene.add_geometry(trimesh.creation.axis())

    if isinstance(entities, list):
        meshes = list()
        for item in entities:
            try:
                new_meshes = item.get_meshes(mesh_type=mesh_type)
            except RuntimeWarning as ex:
                PCG_ROOT_LOGGER.error(
                    'Cannot display {}, message={}'.format(
                        item.name, ex))
                new_meshes = list()
            meshes = meshes + new_meshes
    else:
        meshes = entities.get_meshes(mesh_type=mesh_type)
    if len(meshes) > 0:
        if add_pseudo_color:
            for i in range(len(meshes)):
                meshes[i].visual.face_colors = np.hstack(
                    (255. * np.random.random(3), [alpha * 255]))
        scene.add_geometry(meshes)

    return scene


def plot_occupancy_grid(
        models,
        occupied_thresh=0.65,
        free_thresh=0.196,
        occupied_color=[0, 0, 0],
        free_color=[1, 1, 1],
        unavailable_color=[0.5, 0.5, 0.5],
        output_folder='/tmp',
        output_filename='map.pgm',
        static_models_only=True,
        with_ground_plane=True,
        z_levels=None,
        x_limits=None,
        y_limits=None,
        z_limits=None,
        n_processes=None,
        fig_size=(5, 5),
        fig_size_unit='cm',
        dpi=200,
        axis_x_limits=None,
        axis_y_limits=None,
        exclude_contains=None,
        mesh_type='collision',
        ground_plane_models=None):

    if not MATPLOTLIB_AVAILABLE:
        PCG_ROOT_LOGGER.error('Matplotlib is not available')
        return None

    if mesh_type not in ['collision', 'visual']:
        PCG_ROOT_LOGGER.error(
            'Mesh type must be either collision or visual,'
            ' provided={}'.format(mesh_type))
        return None
    else:
        PCG_ROOT_LOGGER.info(
            'Generating grid map from {} meshes'.format(mesh_type))

    if exclude_contains is not None:
        if not isinstance(exclude_contains, list):
            PCG_ROOT_LOGGER.error('Filter exclude_contains must be a list')
            return None
        for elem in exclude_contains:
            if not isinstance(elem, str):
                PCG_ROOT_LOGGER.error(
                    'Element of filter exclude_contains'
                    ' is not a valid string, value={}'.format(elem))
                return None
        PCG_ROOT_LOGGER.info(
            'Excluding models with names that'
            ' contain={}'.format(exclude_contains))
    else:
        exclude_contains = list()

    if ground_plane_models is None:
        ground_plane_models = list()

    # Create internal function to check if a model should be ignored from the
    # map
    def is_excluded(model_name):
        for item in exclude_contains:
            if item in model_name:
                return True
        return False

    PCG_ROOT_LOGGER.info(
        'Plotting occupancy grid, models={}'.format(
            models.keys()))

    from .generators.occupancy import generate_occupancy_grid

    filtered_models = dict()
    for tag in models:
        if not is_excluded(tag):
            if (models[tag].is_ground_plane or tag in ground_plane_models) \
                    and not with_ground_plane:
                continue
            if not models[tag].static and static_models_only:
                continue
            filtered_models[tag] = models[tag]

    PCG_ROOT_LOGGER.info('Computing model footprints using ray tracing')
    occupancy_output = generate_occupancy_grid(
        filtered_models,
        z_levels=z_levels,
        x_limits=x_limits,
        y_limits=y_limits,
        z_limits=z_limits,
        n_processes=n_processes,
        mesh_type=mesh_type,
        ground_plane_models=ground_plane_models)

    if fig_size_unit == 'cm':
        fig_size_factor = 0.393701
    elif fig_size_unit == 'm':
        fig_size_factor = 39.37
    elif fig_size_unit == 'inch':
        fig_size_factor = 1
    else:
        msg = 'Figure sizes can be given in cm, m or inch, provided={}'.format(
            fig_size_unit)
        PCG_ROOT_LOGGER.error(msg)
        raise ValueError(msg)

    PCG_ROOT_LOGGER.info('Create figure of size={}, unit={}'.format(
        fig_size, fig_size_unit))
    fig = plt.figure(
        dpi=dpi,
        frameon=False)

    fig.set_size_inches(
        fig_size[0] *
        fig_size_factor,
        fig_size[1] *
        fig_size_factor,
        forward=True)
    ax = fig.add_subplot(111)
    fig.subplots_adjust(0, 0, 1, 1)

    min_axis_x = None
    max_axis_x = None

    min_axis_y = None
    max_axis_y = None

    if occupancy_output is not None:
        if with_ground_plane:
            if occupancy_output['ground_plane'] is not None:
                ax.patch.set_facecolor(unavailable_color)
                PCG_ROOT_LOGGER.info(
                    'Adding ground plane models as '
                    'free-space in the occupancy grid')

                patch = descartes.PolygonPatch(
                    occupancy_output['ground_plane'],
                    facecolor=free_color,
                    edgecolor=free_color,
                    alpha=1,
                    linestyle='solid')
                ax.add_patch(patch)

                PCG_ROOT_LOGGER.info(
                    'Ground-plane footprint added'
                    ' to occupancy grid')

                if axis_x_limits is None:
                    if min_axis_x is None:
                        min_axis_x = \
                            occupancy_output['ground_plane'].bounds[0]
                    else:
                        min_axis_x = min(
                            min_axis_x,
                            occupancy_output['ground_plane'].bounds[0])

                    if max_axis_x is None:
                        max_axis_x = \
                            occupancy_output['ground_plane'].bounds[2]
                    else:
                        max_axis_x = max(
                            max_axis_x,
                            occupancy_output['ground_plane'].bounds[2])

                if axis_y_limits is None:
                    if min_axis_y is None:
                        min_axis_y = \
                            occupancy_output['ground_plane'].bounds[1]
                    else:
                        min_axis_y = min(
                            min_axis_y,
                            occupancy_output['ground_plane'].bounds[1])

                    if max_axis_y is None:
                        max_axis_y = \
                            occupancy_output['ground_plane'].bounds[3]
                    else:
                        max_axis_y = max(
                            max_axis_y,
                            occupancy_output['ground_plane'].bounds[3])
            else:
                PCG_ROOT_LOGGER.info(
                    'Ignoring ground plane from the occupancy grid')
                ax.set_facecolor(free_color)
        else:
            PCG_ROOT_LOGGER.info(
                'Ignoring ground plane from the occupancy grid')
            ax.set_facecolor(free_color)

        if 'static' in occupancy_output:
            if len(occupancy_output['static']) > 0:
                PCG_ROOT_LOGGER.info('Adding static models as occupied areas')

                for tag in occupancy_output['static']:
                    if occupancy_output['static'][tag].is_empty:
                        PCG_ROOT_LOGGER.warning(
                            'Model footprint <{}> is empty'.format(tag))
                        continue

                    patch = descartes.PolygonPatch(
                        occupancy_output['static'][tag],
                        facecolor=occupied_color,
                        edgecolor=occupied_color,
                        alpha=1,
                        linestyle='solid')
                    ax.add_patch(patch)
                    PCG_ROOT_LOGGER.info(
                        'Static model <{}> added to'
                        ' occupancy grid'.format(tag))

                    if axis_x_limits is None:
                        if min_axis_x is None:
                            min_axis_x = \
                                occupancy_output['static'][tag].bounds[0]
                        else:
                            min_axis_x = min(
                                min_axis_x,
                                occupancy_output['static'][tag].bounds[0])

                        if max_axis_x is None:
                            max_axis_x = \
                                occupancy_output['static'][tag].bounds[2]
                        else:
                            max_axis_x = max(
                                max_axis_x,
                                occupancy_output['static'][tag].bounds[2])

                    if axis_y_limits is None:
                        if min_axis_y is None:
                            min_axis_y = \
                                occupancy_output['static'][tag].bounds[1]
                        else:
                            min_axis_y = min(
                                min_axis_y,
                                occupancy_output['static'][tag].bounds[1])

                        if max_axis_y is None:
                            max_axis_y = \
                                occupancy_output['static'][tag].bounds[3]
                        else:
                            max_axis_y = max(
                                max_axis_y,
                                occupancy_output['static'][tag].bounds[3])

        if not static_models_only and len(occupancy_output['non_static']) > 0:
            PCG_ROOT_LOGGER.info('Adding non-static models as occupied areas')

            for tag in occupancy_output['non_static']:
                if occupancy_output['non_static'][tag].is_empty:
                    PCG_ROOT_LOGGER.warning(
                        'Model footprint <{}> is empty'.format(tag))
                    continue

                patch = descartes.PolygonPatch(
                    occupancy_output['non_static'][tag],
                    facecolor=occupied_color,
                    edgecolor=occupied_color,
                    alpha=1,
                    linestyle='solid')
                ax.add_patch(patch)
                PCG_ROOT_LOGGER.info(
                    'Non-static model <{}> added to'
                    ' occupancy grid'.format(tag))

                if axis_x_limits is None:
                    if min_axis_x is None:
                        min_axis_x = \
                            occupancy_output['non_static'][tag].bounds[0]
                    else:
                        min_axis_x = min(
                            min_axis_x,
                            occupancy_output['non_static'][tag].bounds[0])

                    if max_axis_x is None:
                        max_axis_x = \
                            occupancy_output['non_static'][tag].bounds[2]
                    else:
                        max_axis_x = max(
                            max_axis_x,
                            occupancy_output['non_static'][tag].bounds[2])

                if axis_y_limits is None:
                    if min_axis_y is None:
                        min_axis_y = \
                            occupancy_output['non_static'][tag].bounds[1]
                    else:
                        min_axis_y = min(
                            min_axis_y,
                            occupancy_output['non_static'][tag].bounds[1])

                    if max_axis_y is None:
                        max_axis_y = \
                            occupancy_output['non_static'][tag].bounds[3]
                    else:
                        max_axis_y = max(
                            max_axis_y,
                            occupancy_output['non_static'][tag].bounds[3])
        else:
            PCG_ROOT_LOGGER.info('Ignore non-static models')

    # ax.set_frame_on(False)
    ax.axis('equal')
    ax.grid(False)
    ax.autoscale(enable=True, axis='both')
    # Removing axis labels
    ax.axes.get_xaxis().set_visible(False)
    ax.axes.get_yaxis().set_visible(False)
    # Removing the axis dark frame
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)

    if None not in [axis_x_limits, min_axis_x]:
        if axis_x_limits is not None:
            ax.set_xlim(axis_x_limits)
        else:
            ax.set_xlim([min_axis_x - 0.1, max_axis_x + 0.1])

    if None not in [axis_y_limits, min_axis_y]:
        if axis_y_limits is not None:
            ax.set_ylim(axis_y_limits)
        else:
            ax.set_ylim([min_axis_y - 0.1, max_axis_y + 0.1])

    fig.canvas.draw()

    if output_folder is not None and os.path.isdir(output_folder):
        if '.pgm' in output_filename:
            filename = os.path.join(output_folder, output_filename)
            filename_svg = os.path.join(
                output_folder, output_filename.replace(
                    '.pgm', '.svg'))
            has_ground_plane = True
            if occupancy_output is not None:
                has_ground_plane = occupancy_output['ground_plane'] is not None

            if with_ground_plane and has_ground_plane:
                color = unavailable_color
            else:
                color = free_color
            plt.savefig(
                filename_svg,
                dpi=dpi,
                bbox_inches='tight',
                facecolor=color,
                edgecolor=color)

            store_fig_as_pgm(
                output_folder,
                output_filename,
                plt.get_current_fig_manager().canvas)

        PCG_ROOT_LOGGER.info('Storing occupancy map at {}'.format(filename))

    return fig


def store_fig_as_pgm(output_folder, output_filename, canvas):
    if not '.pgm' == output_filename[-4::]:
        PCG_ROOT_LOGGER.error(
            'Map filename must have a PGM extension,'
            ' provided={}'.format(output_filename))
        return False
    if not os.path.isdir(output_folder):
        PCG_ROOT_LOGGER.error(
            'Invalid output folder, provided={}'.format(output_folder))
        return False

    import yaml

    canvas = plt.get_current_fig_manager().canvas
    ax = plt.gca()

    x_lims = ax.get_xlim()
    y_lims = ax.get_ylim()

    # Retrieve the figure's size
    ncols, nrows = canvas.get_width_height()

    # Read the RGB values from the plot image
    im = np.fromstring(
        canvas.tostring_rgb(),
        dtype=np.uint8).reshape(
        nrows,
        ncols,
        3)

    with open(os.path.join(output_folder, output_filename), 'wb') as pgm_file:
        # Write magic number
        pgm_file.write('P2\n'.encode())
        # Write the size of the image (width and height)
        pgm_file.write(('{} {}\n'.format(ncols, nrows)).encode())
        # Write the maximum gray value
        pgm_file.write(('{}\n'.format(im.max())).encode())

        # Write the pixels
        for i in range(nrows):
            for j in range(ncols):
                pgm_file.write(('{} '.format(im[i, j].max())).encode())
            pgm_file.write('\n'.encode())

    # Write the map's information on the yaml file
    with open(os.path.join(
            output_folder, output_filename.replace(
                '.pgm', '.yaml')), 'w+') as pgm_info_file:
        pgm_info = dict(
            image=str(output_filename),
            resolution=float((x_lims[1] - x_lims[0]) / im.shape[0]),
            origin=[float(x_lims[0]), float(y_lims[0]), 0.000000],
            negate=0,
            occupied_thresh=0.65,
            free_thresh=0.196)

        yaml.dump(pgm_info, pgm_info_file)

    return True
