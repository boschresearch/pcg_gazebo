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
from ...log import PCG_ROOT_LOGGER
import trimesh
import numpy as np
from .constraint import Constraint


class TangentConstraint(Constraint):
    """Class that allows computation of the closes position
    for a model regarding a reference to have it placed tangent
    to the reference.
    Reference can be a plane or another model, at the moment.

    The input reference types that are supported are

    * `plane`:

    To set a reference plane to which models will be placed tangently,
    the `reference` input must be provided as

    ```python
    reference = dict(
        type='plane',
        args=dict(
            normal=[0, 0, 1], # A 3 element unit vector normal to the plane
            origin=[0, 0, 0]  # The 3D position of the origin of the plane
            )
    )
    ```

    > *Attributes*

    * `LABEL` (*type:* `str`, *value:* `'tangent'`): Name of the
    constraint class
    * `_REFERENCE_TYPES` (*type:* `list`): List of types of references
    that can be used for the computation
    * `_reference` (*type:* `dict`): Arguments of the type of reference
    used.

    > *Input arguments*

    * `reference` (*type:* `dict`): Arguments for the reference used
    for the tangent computation
    * `frame` (*type:* `str`, *default:* `world`): Name of the frame
    of reference with respect to which the poses are going to be
    generated (**not implemented**)
    """
    _LABEL = 'tangent'

    _REFERENCE_TYPES = ['plane']

    def __init__(self, reference, frame='world'):
        """Class constructor."""
        Constraint.__init__(self)

        assert isinstance(
            reference, dict), 'Input reference is not a dictionary'
        assert 'type' in reference and 'args' in reference, \
            'type and args missing from reference definition'
        assert reference['type'] in self._REFERENCE_TYPES, \
            'Invalid reference type'
        PCG_ROOT_LOGGER.info('Creating a tangent constraint')

        self._reference = reference

    def __eq__(self, other):
        if not isinstance(other, TangentConstraint):
            return False
        if other._LABEL != self._LABEL:
            return False
        if self._reference['type'] != other._reference['type']:
            return False
        for tag in self._reference['args']:
            if tag not in other._reference['args']:
                return False
            if other._reference['args'][tag] != self._reference['args'][tag]:
                return False
        return True

    def apply_constraint(self, model):
        """Compute and apply the tangent constraint for the
        provided model using the reference input.

        > *Input arguments*

        * `model` (*type:* `pcg_gazebo.simulation.SimulationModel`):
        Model entity to have its pose adapted so that it is placed
        tangent to the reference
        """
        meshes = model.get_meshes()
        position = np.array([0, 0, 0])
        min_distances = list()

        if self._reference['type'] == 'plane':
            for mesh in meshes:
                d = trimesh.points.point_plane_distance(
                    mesh.vertices,
                    self._reference['args']['normal'],
                    self._reference['args']['origin'])
                min_distances.append(np.min(d))

            position = np.min(min_distances) * \
                np.array(self._reference['args']['normal'])
            model.pose.position = model.pose.position - position
