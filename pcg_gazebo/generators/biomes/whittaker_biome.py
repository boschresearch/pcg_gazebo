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
from .biome import Biome


class WhittakerBiome(Biome):
    def __init__(self):
        super(WhittakerBiome, self).__init__(
            n_moisture_zones=6,
            n_elevation_zones=4)

        biomes = dict(
            snow=[248, 248, 248],
            tundra=[221, 221, 187],
            bare=[187, 187, 187],
            scorched=[153, 153, 153],
            taiga=[204, 212, 187],
            shrubland=[196, 204, 187],
            temperate_desert=[228, 232, 202],
            temperate_rain_forest=[164, 196, 168],
            temperate_deciduous_forest=[180, 201, 169],
            grassland=[196, 212, 170],
            tropical_rain_forest=[156, 187, 169],
            tropical_seasonal_forest=[169, 204, 164],
            subtropical_desert=[233, 221, 199]
        )

        for tag in biomes:
            self.add_biome(tag, color=biomes[tag])

        # Set biomes for elevation #0
        self.add_rule('subtropical_desert', 0, 0)
        self.add_rule('grassland', 1, 0)
        self.add_rule('tropical_seasonal_forest', 2, 0)
        self.add_rule('tropical_seasonal_forest', 3, 0)
        self.add_rule('tropical_rain_forest', 4, 0)
        self.add_rule('tropical_rain_forest', 5, 0)
        # Set biomes for elevation #1
        self.add_rule('temperate_desert', 0, 1)
        self.add_rule('grassland', 1, 1)
        self.add_rule('grassland', 2, 1)
        self.add_rule('temperate_deciduous_forest', 3, 1)
        self.add_rule('temperate_deciduous_forest', 4, 1)
        self.add_rule('temperate_rain_forest', 5, 1)
        # Set biomes for elevation #2
        self.add_rule('temperate_desert', 0, 2)
        self.add_rule('temperate_desert', 1, 2)
        self.add_rule('shrubland', 2, 2)
        self.add_rule('shrubland', 3, 2)
        self.add_rule('taiga', 4, 2)
        self.add_rule('taiga', 5, 2)
        # Set biomes for elevation #3
        self.add_rule('scorched', 0, 3)
        self.add_rule('bare', 1, 3)
        self.add_rule('tundra', 2, 3)
        self.add_rule('snow', 3, 3)
        self.add_rule('snow', 4, 3)
        self.add_rule('snow', 5, 3)

        self.set_min_height(100.0, 3)
        self.set_fade_dist(1, 3)
        self.set_min_height(80.0, 2)
        self.set_fade_dist(1, 2)
        self.set_min_height(10.0, 1)
        self.set_fade_dist(1, 1)
