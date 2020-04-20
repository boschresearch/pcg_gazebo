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

from ..types import XMLScalar


class Mu(XMLScalar):
    """
    Coefficient of friction in the range of [0, 1]

    Args:
        default (float): Coefficient of friction

    Attributes:
        value (float): Stored coefficient of friction
    """
    _NAME = 'mu'
    _TYPE = 'sdf'

    def __init__(self, default=1):
        super(Mu, self).__init__(default, min_value=0)
        self._description = dict(
            ode="(float) Coefficient of friction in the range of [0, 1]")
