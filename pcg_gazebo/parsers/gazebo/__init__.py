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

from .kd import Kd
from .kp import Kp
from .material import Material
from .max_contacts import MaxContacts
from .max_vel import MaxVel
from .min_depth import MinDepth
from .mu1 import Mu1
from .mu2 import Mu2
from .provide_feedback import ProvideFeedback
from .self_collide import SelfCollide
from .stop_cfm import StopCFM
from .stop_erp import StopERP


def get_all_gazebo_element_classes():
    """Get list of all Gazebo element classes."""
    import sys
    import inspect
    from ..types import XMLBase
    output = list()
    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase) and obj._TYPE == 'gazebo':
                output.append(obj)
    return output


def create_gazebo_element(tag, *args):
    """Gazebo element factory.

    > *Input arguments*

    * `tag` (*type:* `str`): Name of the Gazebo element.
    * `args`: Extra arguments for Gazebo element constructor.

    > *Returns*

    Gazebo element if `tag` refers to a valid Gazebo element.
    `None`, otherwise.
    """
    import sys
    import inspect
    from ..types import XMLBase
    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase):
                if tag == obj._NAME and obj._TYPE == 'gazebo':
                    return obj(*args)
    return None


def create_gazebo_type(tag):
    """Return handle of the Gazebo element type.

    > *Input arguments*

    * `tag` (*type:* `str`): Name of the Gazebo element.

    > *Returns*

    Gazebo element type if `tag` is valid, `None` otherwise`.
    """
    import sys
    import inspect
    from ..types import XMLBase
    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase):
                if tag == obj._NAME and obj._TYPE == 'gazebo':
                    return obj
    return None


def is_gazebo_element(obj):
    """Test if XML element is an Gazebo element."""
    from ..types import XMLBase
    return obj.__class__ in XMLBase.__subclasses__() and \
        obj._TYPE == 'gazebo'


__all__ = [
    'Kd',
    'Kp',
    'Material',
    'MaxContacts',
    'MaxVel',
    'MinDepth',
    'Mu1',
    'Mu2',
    'ProvideFeedback',
    'SelfCollide',
    'StopCFM',
    'StopERP'
]
