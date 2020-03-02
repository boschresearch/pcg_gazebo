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
from ...simulation import Light


class Sun(Light):
    def __init__(self, name='sun', pose=[0, 0, 10, 0, 0, 0]):
        super(Sun, self).__init__(
            name=name,
            pose=pose,
            type='directional')
        self.cast_shadows = True
        self.diffuse = [0.8, 0.8, 0.8, 1.0]
        self.specular = [0.2, 0.2, 0.2, 1.0]
        self.direction = [-0.5, 0.1, -0.9]
        self.set_attenuation(
            range=1000,
            constant=0.9,
            linear=0.01,
            quadratic=0.001)
