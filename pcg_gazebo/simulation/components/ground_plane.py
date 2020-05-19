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
from ..model import SimulationModel
from ..link import Link
from ..properties import Visual, \
    Collision


class GroundPlane(SimulationModel):
    def __init__(self, name='ground_plane'):
        super(GroundPlane, self).__init__(
            name=name)

        # Set ground plane as a static model
        self.static = True
        # Auto disable enabled
        self.allow_auto_disable = True
        # Disable self-collide
        self.self_collide = False

        # Add ground plane link
        link = Link()
        # Set plane parameters
        plane_params = dict(
            size=[100, 100],
            normal=[0, 0, 1])
        visual = Visual(
            name='visual',
            cast_shadows=False,
            transparency=0,
            geometry_type='plane',
            geometry_args=plane_params)
        visual.set_color(r=.175, g=.175, b=.175, a=1.0)
        link.add_visual(visual)

        collision = Collision(
            name='collision',
            mu=100.0,
            mu2=50.0,
            slip1=0,
            slip2=0,
            collide_bitmask=65535,
            max_contacts=20,
            geometry_type='plane',
            geometry_args=plane_params)
        link.add_collision(collision)
        self.add_link(name='link', link=link)
