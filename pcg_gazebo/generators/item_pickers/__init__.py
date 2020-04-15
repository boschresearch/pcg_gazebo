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
from ._picker import _Picker
from .random_picker import RandomPicker  # noqa: F401
from .size_picker import SizePicker  # noqa: F401
from .roulette_wheel_picker import RouletteWheelPicker  # noqa: F401


def create_picker(tag, **kwargs):
    """Picker factory that returns the picker according
    to its `LABEL` definition. It returns `None` if the picker name
    is invalid.

    > *Input parameters*

    * `tag` (*type:* `str`): Name of the picker class
    * `kwargs`: Inputs for the picker class constructor
    """
    import inspect
    from ...log import PCG_ROOT_LOGGER

    for obj in _Picker.__subclasses__():
        if inspect.isclass(obj):
            if issubclass(obj, _Picker):
                if tag == obj._LABEL:
                    PCG_ROOT_LOGGER.info(
                        'Creating picker: {}'.format(
                            obj._LABEL))
                    return obj(**kwargs)
    PCG_ROOT_LOGGER.error('Picker {} does not exist'.format(tag))
    return None


def get_picker_tags():
    import inspect
    tags = list()
    for obj in _Picker.__subclasses__():
        if inspect.isclass(obj):
            if issubclass(obj, _Picker):
                tags.append(obj._LABEL)
    return tags


___all__ = [
    'create_picker',
    'get_picker_tags',
    '_Picker',
    'RandomPicker',
    'SizePicker',
    'RouletteWheelPicker'
]
