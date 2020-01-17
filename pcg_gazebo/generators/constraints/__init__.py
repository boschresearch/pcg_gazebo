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
"""Spatial constraints for the placement of simulation
entities into the world.
"""
from .constraint import Constraint
from .workspace_constraint import WorkspaceConstraint
from .tangent_constraint import TangentConstraint

__all__ = [
    'Constraint',
    'WorkspaceConstraint',
    'TangentConstraint'
]


def create_constraint(tag, **kwargs):
    """Constraint factory that returns the constraint according
    to its `LABEL` definition. It returns `None` if the constraint name
    is invalid.

    > *Input parameters*

    * `tag` (*type:* `str`): Name of the constraint class
    * `kwargs`: Inputs for the constraint class constructor
    """
    import inspect
    from ...log import PCG_ROOT_LOGGER

    for obj in Constraint.__subclasses__():
        if inspect.isclass(obj):
            if issubclass(obj, Constraint):
                if tag == obj._LABEL:
                    return obj(**kwargs)
    PCG_ROOT_LOGGER.error('Constraint {} does not exist'.format(tag))
    return None
