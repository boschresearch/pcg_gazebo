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
import os
from .components import Sun, GroundPlane
from ..utils import load_yaml
from ..simulation import SimulationModel, ModelGroup, Light, \
    add_custom_gazebo_resource_path
from ._collection_manager import _CollectionManager
from ..log import PCG_ROOT_LOGGER


class AssetsManager(_CollectionManager):
    """Assets manager containing all valid Gazebo models and model group
    generators. This collection should be initialized as a singleton object
    in order to have a single source of model to all instances of engines,
    model and world generators.
    The asset types allowed to be added are:

    * `pcg_gazebo.simulation.SimulationModel`: Description for a model
    * `pcg_gazebo.simulation.Light`: Description for light sources
    * `pcg_gazebo.simulation.ModelGroup`: Group of models and light sources
    * `pcg_gazebo.generators.ModelGroupGenerator`: Dynamic model group
    generator
    * `dict`: Input configuration of the `creators` factory methods for `box`,
    `sphere`, `cylinder` and `mesh` models, for an instance of
    `pcg_gazebo.simulation.Light`, or an instance of
    `pcg_gazebo.generators.ModelGroupGenerator`
    * `str`: Name of an existing Gazebo model that can be found in the
    Gazebo resources path
    """

    def __init__(self):
        super(AssetsManager, self).__init__()
        self._ground_plane_assets = list()
        self.init()

    @staticmethod
    def get_instance():
        """Return singleton instance of the `AssetsMananger`"""
        if AssetsManager._INSTANCE is None:
            AssetsManager._INSTANCE = AssetsManager()
        AssetsManager._INSTANCE.init()
        return AssetsManager._INSTANCE

    @property
    def tags(self):
        """`list`: List of strings with all asset tags"""
        from ..simulation import get_gazebo_model_names
        return list(self._collection.keys()) + list(get_gazebo_model_names())

    @property
    def ground_planes(self):
        """`list`: List of strings with tags of ground plane models"""
        return self._ground_plane_assets

    def init(self):
        if 'sun' not in self.tags:
            self.add(description=Sun(), tag='sun')
        if 'ground_plane' not in self.tags:
            self.add(description=GroundPlane(), tag='ground_plane')
            self.set_asset_as_ground_plane('ground_plane')

    def is_model(self, tag):
        """Return if asset identified by `tag` is an instance of
        `pcg_gazebo.simulation.SimulationModel`.

        > *Input arguments*

        * `tag` (*type:* `str`): Tag of the asset.
        """
        if tag in self._collection:
            return isinstance(self._collection[tag], SimulationModel)
        return False

    def is_light(self, tag):
        """Return if asset identified by `tag` is an instance of
        `pcg_gazebo.simulation.Light`.

        > *Input arguments*

        * `tag` (*type:* `str`): Tag of the asset.
        """
        if tag in self._collection:
            return isinstance(self._collection[tag], Light)
        return False

    def is_model_group(self, tag):
        """Return if asset identified by `tag` is an instance of
        `pcg_gazebo.simulation.ModelGroup`.

        > *Input arguments*

        * `tag` (*type:* `str`): Tag of the asset.
        """
        if tag in self._collection:
            return isinstance(self._collection[tag], ModelGroup)
        return False

    def is_gazebo_model(self, tag):
        """Return if asset identified by `tag` is a Gazebo model
        found in Gazebo's resources path.

        > *Input arguments*

        * `tag` (*type:* `str`): Tag of the asset.
        """
        from ..simulation import is_gazebo_model
        return is_gazebo_model(tag)

    def is_model_group_generator(self, tag):
        """Return if asset identified by `tag` is an instance of
        `pcg_gazebo.generators.ModelGroupGenerator`.

        > *Input arguments*

        * `tag` (*type:* `str`): Tag of the asset.
        """
        from .model_group_generator import ModelGroupGenerator
        if tag in self._collection:
            return isinstance(self._collection[tag], ModelGroupGenerator)
        return False

    def is_ground_plane(self, tag):
        """Return if asset identified by `tag` is flagged as a ground
        plane model.

        > *Input arguments*

        * `tag` (*type:* `str`): Tag of the asset.
        """
        return tag in self._ground_plane_assets

    def is_factory_input(self, tag):
        """Return if asset identified by `tag` is a `dict` containing
        the inputs for a `pcg_gazebo.generators.creators` factory method
        to create a `box`, `sphere`, `cylinder` or `mesh` model.

        > *Input arguments*

        * `tag` (*type:* `str`): Tag of the asset.
        """
        if tag in self._collection:
            if not isinstance(self._collection[tag], dict):
                return False
            if 'type' not in self._collection[tag] or \
                    'args' not in self._collection[tag]:
                return False
            if self._collection[tag]['type'] in \
                    ['box', 'sphere', 'cylinder', 'mesh']:
                return True
        return False

    def add(
            self,
            description,
            tag=None,
            type=None,
            parameters=None,
            include_dir=None,
            overwrite=True):
        """Add new asset to the collection.

        > *Input arguments*

        * `description` (*type:* `str`, `dict`,
        `pcg_gazebo.simulation.SimulationModel`,
        `pcg_gazebo.simulation.Light`, `pcg_gazebo.simulation.ModelGroup` or
        `pcg_gazebo.generators.ModelGroupGenerator`): Model description.
        * `tag` (*type:* `str`, *default:* `None`): Asset's tag. If `None`
        is provided, the input `description` must have an attribute `name`
        which will be used as a tag, otherwise the function returns `False`.
        * `type` (*type:* `str`, *default:* `None`): When the provided
        description is `dict`, the type of asset that must be generated
        with the `dict` input must be then provided as either `factory`,
        `model_generator` or `light`.

        > *Returns*

        `True`, if asset could be added to the collection.
        """
        from ..parsers import parse_sdf, parse_urdf, urdf2sdf
        from .model_group_generator import ModelGroupGenerator
        if self.has_element(tag) and not overwrite:
            PCG_ROOT_LOGGER.warning(
                'Asset with tag <{}> already exists, tags='.format(
                    tag, self.tags))
            return False
        assert isinstance(description, SimulationModel) or \
            isinstance(description, Light) or \
            isinstance(description, ModelGroup) or \
            isinstance(description, ModelGroupGenerator) or \
            isinstance(description, dict) or isinstance(description, str), \
            'Invalid asset type, options are=SimulationModel, Light,' \
            ' ModelGroup, ModelGroupGenerator, dict, received={}'.format(
                type(description))
        if hasattr(description, 'name'):
            if tag is None:
                tag = description.name
            self._collection[tag] = description
        elif isinstance(description, str):
            if os.path.isfile(description):
                ext = description.split('.')[-1]
                if ext == 'sdf':
                    sdf = parse_sdf(description)
                elif ext == 'urdf':
                    sdf = urdf2sdf(parse_urdf(description))
                else:
                    return False
                # TODO Add processing Jinja templates to generate assets
                if sdf.xml_element_name == 'model':
                    try:
                        model = SimulationModel.from_sdf(sdf)
                        if model is not None:
                            return self.add(model, tag=tag)
                        else:
                            PCG_ROOT_LOGGER.error(
                                'No model found in file={}'.format(
                                    description))
                            return False
                    except ValueError as ex:
                        PCG_ROOT_LOGGER.error(
                            'Failed to load a model from '
                            'file={}, message={}'.format(
                                description, ex))
                        return False
                elif sdf.xml_element_name == 'sdf':
                    models = list() if sdf.models is None else sdf.models
                    lights = list() if sdf.lights is None else sdf.lights
                    if len(models) == 1 and len(lights) == 0:
                        return self.add(
                            SimulationModel.from_sdf(
                                models[0]), tag=tag)
                    elif len(models) == 0 and len(lights) == 1:
                        return self.add(Light.from_sdf(sdf.lights[0]), tag=tag)
                    elif len(models) > 0 or len(lights) > 0:
                        return self.add(ModelGroup.from_sdf(sdf), tag=tag)
            else:
                # The string must be otherwise an already existant
                # Gazebo model
                if not self.is_gazebo_model(tag):
                    PCG_ROOT_LOGGER.error(
                        'Input string does not refer to a Gazebo'
                        ' model, value={}'.format(description))
                    return False
                self._collection[tag] = description
        else:
            if type is None or type == 'factory':
                if not isinstance(description, dict):
                    PCG_ROOT_LOGGER.error(
                        'Factory model constructor must be a '
                        'dict, value={}'.format(description))
                    return False
                if 'type' not in description or 'args' not in description:
                    PCG_ROOT_LOGGER.error('Factory model constructor requires '
                                          'type and args inputs')
                    return False
                if description['type'] not in \
                        ['box', 'sphere', 'cylinder', 'mesh']:
                    PCG_ROOT_LOGGER.error(
                        'Type of factory model constructor must be either'
                        ' box, sphere, cylinder or mesh, received={}'.format(
                            description['type']))
                    return False
                type = 'factory'
                self._collection[tag] = description
                PCG_ROOT_LOGGER.info('Added model factory <{}>'.format(tag))
            elif type in ['model_generator', 'light']:
                if type == 'model_generator':
                    self._collection[tag] = ModelGroupGenerator(
                        name=tag, **description)
                    PCG_ROOT_LOGGER.info(
                        'Added model group generator <{}>'.format(tag))
                else:
                    self._collection[tag] = Light.from_dict(description)
                    PCG_ROOT_LOGGER.info('Added light <{}>'.format(tag))
            else:
                return False

        PCG_ROOT_LOGGER.info('New asset added={}'.format(tag))
        return True

    def get(self, tag, *args, **kwargs):
        """Return an asset reference by `tag`.

        > *Input arguments*

        * `tag` (*type:* `str`): Tag of the asset. In case `tag` is
        referencing a `pcg_gazebo.generators.ModelGroupGenerator`,
        additional inputs to run the engines can be provided using
        `*args` and `**kwargs`.

        > *Returns*

        `pcg_gazebo.simulation.SimulationModel` or
        `pcg_gazebo.simulation.ModelGroup`. `None`, if `tag` is invalid.
        """
        model = None
        if self.is_factory_input(tag):
            from .creators import config2models
            model = SimulationModel.from_sdf(
                config2models(self._collection[tag])[0])
            model.name = tag
        elif self.is_model_group_generator(tag):
            model = self._collection[tag].run(group_name=tag, *args, **kwargs)
        elif self.is_model(tag) or self.is_light(tag):
            model = self._collection[tag].copy()
        elif self.is_model_group(tag):
            return self._collection[tag]
        elif self.is_gazebo_model(tag):
            model = None
            try:
                model = SimulationModel.from_gazebo_model(tag)
            except ValueError:
                model = ModelGroup.from_gazebo_model(tag)

        if model is not None and not self.is_light(tag):
            model.is_ground_plane = self.is_ground_plane(tag)
        return model

    def set_asset_as_ground_plane(self, tag):
        """Flag a model asset as part of the ground plane. This procedure will
        affect the collision checks during the automatic placement of models in
        the world using the placement engines.

        > *Input arguments*

        * `tag` (*type:* `str`): Name of the model asset
        """
        assert isinstance(tag, str), 'Input tag must be a string'
        if tag not in self._collection and not self.is_gazebo_model(tag):
            PCG_ROOT_LOGGER.error(
                'Asset <{}> not in the list of assets'.format(tag))
            return False
        else:
            if tag not in self._ground_plane_assets:
                self._ground_plane_assets.append(tag)
            return True

    def from_dict(self, config):
        """Read assets from an input `dict`. The dictionary should have a list of
        asset descriptions under the tag `assets` and, if necessary, a list of
        strings referring to models that must be flagged as ground plane under
        the tag `ground_plane`.

        > *Input arguments*

        * `config` (*type:* `data_type`, *default:* `data`):
        Parameter description

        > *Returns*

        Description of return values
        """
        assert isinstance(config, dict), 'Input must be a dictionary'

        if 'assets' in config:
            assert isinstance(config['assets'], list), \
                'Assets must be provided as a list'
            for elem in config['assets']:
                if not self.add(**elem):
                    PCG_ROOT_LOGGER.error('Error adding asset={}'.format(elem))

        # Setting assets as ground plane
        if 'ground_plane' in config:
            assert isinstance(config['ground_plane'], list), \
                'ground_plane input for assets manager' \
                ' must be a list of strings'
            for tag in config['ground_plane']:
                if not self.set_asset_as_ground_plane(tag):
                    PCG_ROOT_LOGGER.error(
                        'Error setting asset <{}> as ground plane'.format(tag))

    def from_yaml(self, filename):
        """Load the assets from a YAML file.

        > *Input arguments*

        * `filename` (*type:* `str`): YAML filename.
        """
        config = load_yaml(filename)
        self.from_dict(config)

    def has_element(self, tag):
        """Return `True` if an element for `tag` exists.

        > *Input arguments*

        * `tag` (*type:* `str`): Tag of the element.
        """
        from ..simulation import get_gazebo_model_names
        if tag in self._collection:
            return True
        if tag in get_gazebo_model_names():
            return True
        return False

    @staticmethod
    def add_custom_gazebo_resource_path(dir_path):
        return add_custom_gazebo_resource_path(dir_path)
