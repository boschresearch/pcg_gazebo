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
"""Pose generator engine definitions that compute the pose of the
models according to pre-defined rules.
"""
from .engine import Engine
from .fixed_pose_engine import FixedPoseEngine
from .random_pose_engine import RandomPoseEngine
from .pattern_engine import PatternEngine


def create_engine(tag, **kwargs):
    """Engine factory that returns the engine according
    to its `LABEL` definition. It returns `None` if the engine name
    is invalid.

    > *Input parameters*

    * `tag` (*type:* `str`): Name of the engine class
    * `kwargs`: Inputs for the engine class constructor
    """
    import inspect
    from ...log import PCG_ROOT_LOGGER

    for obj in Engine.__subclasses__():
        if inspect.isclass(obj):
            if issubclass(obj, Engine):
                if tag == obj._LABEL:
                    PCG_ROOT_LOGGER.info(
                        'Creating engine: {}'.format(
                            obj._LABEL))
                    return obj(**kwargs)
    PCG_ROOT_LOGGER.error('Engine {} does not exist'.format(tag))
    return None


__all__ = [
    'create_engine',
    'Engine',
    'FixedPoseEngine',
    'RandomPoseEngine',
    'PatternEngine'
]
