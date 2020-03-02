# Copyright (c) 2020 - The Procedural Generation for Gazebo authors
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

from .rule import Rule
from .fixed_value import FixedValue
from .from_set import FromSet
from .random import Random
from .uniform import Uniform
from .within_workspace import WithinWorkspace


def create_rule(tag=None, **kwargs):
    """Rule factory that returns the engine according
    to its `LABEL` definition. It returns `None` if the engine name
    is invalid.

    > *Input parameters*

    * `tag` (*type:* `str`): Name of the engine class
    * `kwargs`: Inputs for the engine class constructor
    """
    import inspect
    from ...log import PCG_ROOT_LOGGER

    if 'policy' in kwargs and tag is None:
        rule_args = dict()
        assert 'name' in kwargs['policy']
        tag = kwargs['policy']['name']

        assert 'args' in kwargs['policy']

        if isinstance(kwargs['policy']['args'], dict):
            rule_args.update(kwargs['policy']['args'])
        elif tag == 'workspace':
            rule_args.update(dict(workspace=kwargs['policy']['args']))
        elif tag == 'value':
            rule_args.update(dict(value=kwargs['policy']['args']))

        if tag == 'choice':
            tag = 'from_set'

        assert 'dofs' in kwargs
        rule_args['dofs'] = kwargs['dofs']
    else:
        rule_args = kwargs

    for obj in Rule.__subclasses__():
        if inspect.isclass(obj):
            if issubclass(obj, Rule):
                if tag == obj._NAME:
                    PCG_ROOT_LOGGER.info(
                        'Creating engine: {}'.format(
                            obj._NAME))
                    return obj(**rule_args)
    PCG_ROOT_LOGGER.error('Rule {} does not exist'.format(tag))
    return None


def get_rule_parameters(tag):
    import inspect
    from ...log import PCG_ROOT_LOGGER

    for obj in Rule.__subclasses__():
        if inspect.isclass(obj):
            if issubclass(obj, Rule):
                if tag == obj._NAME:
                    return obj.example()
    PCG_ROOT_LOGGER.error('Rule {} does not exist'.format(tag))
    return None


def get_rule_tags():
    import inspect
    tags = list()
    for obj in Rule.__subclasses__():
        if inspect.isclass(obj):
            if issubclass(obj, Rule):
                tags.append(obj._NAME)
    return tags


__all__ = [
    'create_rule',
    'Rule',
    'FixedValue',
    'FromSet',
    'Random',
    'Uniform',
    'WithinWorkspace'
]
