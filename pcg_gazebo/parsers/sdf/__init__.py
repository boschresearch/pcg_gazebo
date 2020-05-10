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

from .accel import Accel
from .acceleration import Acceleration
from .accuracy import Accuracy
from .angle import Angle
from .actor import Actor
from .allow_auto_disable import AllowAutoDisable
from .altimeter import Altimeter
from .always_on import AlwaysOn
from .ambient import Ambient
from .angular_velocity import AngularVelocity
from .angular import Angular
from .animation import Animation
from .atmosphere import Atmosphere
from .attenuation import Attenuation
from .auto_start import AutoStart
from .axis import Axis
from .axis2 import Axis2
from .background import Background
from .bias_mean import BiasMean
from .bias_stddev import BiasStdDev
from .blend import Blend
from .bounce import Bounce
from .box import Box
from .bullet import Bullet
from .camera import Camera
from .cast_shadows import CastShadows
from .category_bitmask import CategoryBitmask
from .center import Center
from .cfm_damping import CFMDamping
from .cfm import CFM
from .child import Child
from .clip import Clip
from .clouds import Clouds
from .coefficient import Coefficient
from .collide_bitmask import CollideBitmask
from .collide_without_contact_bitmask import CollideWithoutContactBitmask
from .collide_without_contact import CollideWithoutContact
from .collision import Collision
from .color import Color
from .constant import Constant
from .constraints import Constraints
from .contact_max_correcting_vel import ContactMaxCorrectingVel
from .contact_surface_layer import ContactSurfaceLayer
from .contact import Contact
from .cylinder import Cylinder
from .damping import Damping
from .delay_start import DelayStart
from .deletions import Deletions
from .density import Density
from .depth_camera import DepthCamera
from .diffuse import Diffuse
from .direction import Direction
from .dissipation import Dissipation
from .distortion import Distortion
from .dynamic_friction import DynamicFriction
from .dynamics import Dynamics
from .effort import Effort
from .elastic_modulus import ElasticModulus
from .elevation import Elevation
from .emissive import Emissive
from .empty import Empty
from .enable_wind import EnableWind
from .end import End
from .erp import ERP
from .fade_dist import FadeDist
from .falloff import FallOff
from .far import Far
from .fdir1 import FDir1
from .filename import Filename
from .fog import Fog
from .force_torque import ForceTorque
from .format import Format
from .frame import Frame
from .friction_model import FrictionModel
from .friction import Friction
from .friction2 import Friction2
from .fullscreen import Fullscreen
from .geometry import Geometry
from .granularity import Granularity
from .gravity import Gravity
from .grid import Grid
from .gui import GUI
from .heading_deg import HeadingDeg
from .height import Height
from .heightmap import Heightmap
from .horizontal_fov import HorizontalFOV
from .horizontal import Horizontal
from .humidity import Humidity
from .image import Image
from .imu import IMU
from .include import Include
from .inertia import Inertia
from .inertial import Inertial
from .inherit_yaw import InheritYaw
from .initial_position import InitialPosition
from .inner_angle import InnerAngle
from .insertions import Insertions
from .interpolate_x import InterpolateX
from .iterations import Iterations
from .iters import Iters
from .ixx import IXX
from .ixy import IXY
from .ixz import IXZ
from .iyy import IYY
from .iyz import IYZ
from .izz import IZZ
from .joint import Joint
from .k1 import K1
from .k2 import K2
from .k3 import K3
from .kd import Kd
from .kinematic import Kinematic
from .kp import Kp
from .laser_retro import LaserRetro
from .latitude_deg import LatitudeDeg
from .length import Length
from .light import Light
from .lighting import Lighting
from .limit import Limit
from .linear_acceleration import LinearAcceleration
from .linear_velocity import LinearVelocity
from .linear import Linear
from .link import Link
from .localization import Localization
from .longitude_deg import LongitudeDeg
from .loop import Loop
from .lower import Lower
from .magnetic_field import MagneticField
from .mass import Mass
from .material import Material
from .max_angle import MaxAngle
from .max_contacts import MaxContacts
from .max_dist import MaxDist
from .max_step_size import MaxStepSize
from .min import Min
from .max_transient_velocity import MaxTransientVelocity
from .max_vel import MaxVel
from .max import Max
from .mean_size import MeanSize
from .mean import Mean
from .measure_direction import MeasureDirection
from .mesh import Mesh
from .min_angle import MinAngle
from .min_depth import MinDepth
from .min_dist import MinDist
from .min_height import MinHeight
from .min_step_size import MinStepSize
from .model import Model
from .mu import Mu
from .mu2 import Mu2
from .must_be_loop_joint import MustBeLoopJoint
from .name import Name
from .near import Near
from .noise import Noise
from .normal_map import NormalMap
from .normal import Normal
from .ode import ODE
from .orientation_reference_frame import OrientationReferenceFrame
from .origin_visual import OriginVisual
from .outer_angle import OuterAngle
from .output import Output
from .override_impact_capture_velocity import \
    OverrideImpactCaptureVelocity
from .override_stiction_transition_velocity import \
    OverrideStictionTransitionVelocity
from .p1 import P1
from .p2 import P2
from .parent import Parent
from .patch_radius import PatchRadius
from .path import Path
from .physics import Physics
from .plane import Plane
from .plastic_coef_restitution import PlasticCoefRestitution
from .plastic_impact_velocity import PlasticImpactVelocity
from .plugin import Plugin
from .point import Point
from .poissons_ratio import PoissonsRatio
from .polyline import Polyline
from .pos import Pos
from .pose import Pose
from .precision import Precision
from .precon_iters import PreConIters
from .pressure import Pressure
from .projection_type import ProjectionType
from .provide_feedback import ProvideFeedback
from .quadratic import Quadratic
from .radius import Radius
from .range import Range
from .rate import Rate
from .ray import Ray
from .real_time_factor import RealTimeFactor
from .real_time_update_rate import RealTimeUpdateRate
from .real_time import RealTime
from .resolution import Resolution
from .restitution_coefficient import RestitutionCoefficient
from .rolling_friction import RollingFriction
from .samples import Samples
from .sampling import Sampling
from .save import Save
from .scale import Scale
from .scan import Scan
from .scene import Scene
from .script import Script
from .sdf import SDF
from .self_collide import SelfCollide
from .sensor import Sensor
from .shader import Shader
from .shadows import Shadows
from .sim_time import SimTime
from .simbody import Simbody
from .size import Size
from .skin import Skin
from .sky import Sky
from .slip import Slip
from .slip1 import Slip1
from .slip2 import Slip2
from .soft_cfm import SoftCFM
from .soft_erp import SoftERP
from .solver import Solver
from .sor import Sor
from .specular import Specular
from .speed import Speed
from .sphere import Sphere
from .spherical_coordinates import SphericalCoordinates
from .split_impulse_penetration_threshold import \
    SplitImpulsePenetrationThreshold
from .split_impulse import SplitImpulse
from .spot import Spot
from .spring_reference import SpringReference
from .spring_stiffness import SpringStiffness
from .start import Start
from .state import State
from .static_friction import StaticFriction
from .static import Static
from .stddev import StdDev
from .stiffness import Stiffness
from .submesh import SubMesh
from .sunrise import Sunrise
from .sunset import Sunset
from .surface_model import SurfaceModel
from .surface_radius import SurfaceRadius
from .surface import Surface
from .temperature_gradient import TemperatureGradient
from .temperature import Temperature
from .texture import Texture
from .threshold import Threshold
from .time import Time
from .topic import Topic
from .torsional import Torsional
from .track_visual import TrackVisual
from .trajectory import Trajectory
from .transparency import Transparency
from .type import Type
from .update_rate import UpdateRate
from .upper import Upper
from .urdf import URDF
from .uri import URI
from .use_dynamic_moi_rescaling import UseDynamicMOIRescaling
from .use_model_frame import UseModelFrame
from .use_parent_model_frame import UseParentModelFrame
from .use_patch_radius import UsePatchRadius
from .use_terrain_paging import UseTerrainPaging
from .velocity import Velocity
from .vertical_position import VerticalPosition
from .vertical_velocity import VerticalVelocity
from .vertical import Vertical
from .view_controller import ViewController
from .viscous_friction import ViscousFriction
from .visual import Visual
from .visualize import Visualize
from .wall_time import WallTime
from .waypoint import Waypoint
from .width import Width
from .wind import Wind
from .world_frame_orientation import WorldFrameOrientation
from .world import World
from .wrench import Wrench
from .x import X
from .xyz import XYZ
from .y import Y
from .z import Z


def get_all_sdf_element_classes():
    import sys
    import inspect
    from ..types import XMLBase
    output = list()

    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase) and obj._TYPE == 'sdf':
                output.append(obj)
    return output


def create_sdf_element(tag, *args):
    import sys
    import inspect
    from ..types import XMLBase

    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase):
                if tag == obj._NAME and obj._TYPE == 'sdf':
                    return obj(*args)
    return None


def create_sdf_type(tag):
    import sys
    import inspect
    from ..types import XMLBase

    current_module = sys.modules[__name__]
    for name, obj in inspect.getmembers(current_module):
        if inspect.isclass(obj):
            if issubclass(obj, XMLBase):
                if tag == obj._NAME and obj._TYPE == 'sdf':
                    return obj
    return None


def is_sdf_element(obj):
    from ..types import XMLBase
    return obj.__class__ in XMLBase.__subclasses__() and \
        obj._TYPE == 'sdf'


__all__ = [
    'get_all_sdf_element_classes',
    'create_sdf_element',
    'create_sdf_type',
    'is_sdf_element',
    'Accel',
    'Acceleration',
    'Accuracy',
    'Angle',
    'Actor',
    'AllowAutoDisable',
    'Altimeter',
    'AlwaysOn',
    'Ambient',
    'AngularVelocity',
    'Angular',
    'Animation',
    'Atmosphere',
    'Attenuation',
    'AutoStart',
    'Axis',
    'Axis2',
    'Background',
    'BiasMean',
    'BiasStdDev',
    'Blend',
    'Bounce',
    'Box',
    'Bullet',
    'Camera',
    'CastShadows',
    'CategoryBitmask',
    'Center',
    'CFMDamping',
    'CFM',
    'Child',
    'Clip',
    'Clouds',
    'Coefficient',
    'CollideBitmask',
    'CollideWithoutContact',
    'CollideWithoutContactBitmask',
    'Collision',
    'Color',
    'Constant',
    'Constraints',
    'Contact',
    'ContactMaxCorrectingVel',
    'ContactSurfaceLayer',
    'Cylinder',
    'Damping',
    'DelayStart',
    'Deletions',
    'Density',
    'DepthCamera',
    'Diffuse',
    'Direction',
    'Dissipation',
    'Distortion',
    'DynamicFriction',
    'Dynamics',
    'Effort',
    'ElasticModulus',
    'Elevation',
    'Emissive',
    'Empty',
    'EnableWind',
    'End',
    'ERP',
    'FadeDist',
    'FallOff',
    'Far',
    'FDir1',
    'Filename',
    'Fog',
    'ForceTorque',
    'Format',
    'Frame',
    'Friction',
    'Friction2',
    'Fullscreen',
    'FrictionModel',
    'Geometry',
    'Granularity',
    'Gravity',
    'Grid',
    'GUI',
    'HeadingDeg',
    'Height',
    'Heightmap',
    'Horizontal',
    'HorizontalFOV',
    'Humidity',
    'Image',
    'IMU',
    'Include',
    'Inertia',
    'Inertial',
    'InheritYaw',
    'InitialPosition',
    'InnerAngle',
    'Insertions',
    'InterpolateX',
    'Iterations',
    'Iters',
    'IXX',
    'IXY',
    'IXZ',
    'IYY',
    'IYZ',
    'IZZ',
    'Joint',
    'K1',
    'K2',
    'K3',
    'Kd',
    'Kinematic',
    'Kp',
    'LaserRetro',
    'LatitudeDeg',
    'Length',
    'Light',
    'Lighting',
    'Limit',
    'Linear',
    'LinearAcceleration',
    'LinearVelocity',
    'Link',
    'Localization',
    'LongitudeDeg',
    'Loop',
    'Lower',
    'MagneticField',
    'Mass',
    'Material',
    'Max',
    'MaxAngle',
    'MaxContacts',
    'MaxDist',
    'MaxStepSize',
    'MaxTransientVelocity',
    'MaxVel',
    'Mean',
    'MeanSize',
    'MeasureDirection',
    'Mesh',
    'Min',
    'MinAngle',
    'MinDepth',
    'MinDist',
    'MinHeight',
    'MinStepSize',
    'Model',
    'Mu',
    'Mu2',
    'MustBeLoopJoint',
    'Name',
    'Near',
    'Noise',
    'Normal',
    'NormalMap',
    'ODE',
    'OrientationReferenceFrame',
    'OriginVisual',
    'OuterAngle',
    'Output',
    'OverrideImpactCaptureVelocity',
    'OverrideStictionTransitionVelocity',
    'P1',
    'P2',
    'Parent',
    'PatchRadius',
    'Path',
    'Physics',
    'Plane',
    'PlasticCoefRestitution',
    'PlasticImpactVelocity',
    'Plugin',
    'Point',
    'PoissonsRatio',
    'Polyline',
    'Pos',
    'Pose',
    'Precision',
    'PreConIters',
    'Pressure',
    'ProjectionType',
    'ProvideFeedback',
    'Quadratic',
    'Radius',
    'Range',
    'Rate',
    'Ray',
    'RealTimeFactor',
    'RealTimeUpdateRate',
    'RealTime',
    'Resolution',
    'RestitutionCoefficient',
    'RollingFriction',
    'Samples',
    'Sampling',
    'Save',
    'Scale',
    'Scan',
    'Scene',
    'Script',
    'SDF',
    'SelfCollide',
    'Sensor',
    'Shader',
    'Shadows',
    'SimTime',
    'Simbody',
    'Size',
    'Skin',
    'Sky',
    'Slip',
    'Slip1',
    'Slip2',
    'SoftCFM',
    'SoftERP',
    'Solver',
    'Sor',
    'Specular',
    'Speed',
    'Sphere',
    'SphericalCoordinates',
    'SplitImpulse',
    'SplitImpulsePenetrationThreshold',
    'Spot',
    'SpringReference',
    'SpringStiffness',
    'Start',
    'State',
    'Static',
    'StaticFriction',
    'StdDev',
    'Stiffness',
    'SubMesh',
    'Sunrise',
    'Sunset',
    'SurfaceModel',
    'Surface',
    'SurfaceRadius',
    'TemperatureGradient',
    'Temperature',
    'Texture',
    'Threshold',
    'Time',
    'Topic',
    'Torsional',
    'TrackVisual',
    'Trajectory',
    'Transparency',
    'Type',
    'UpdateRate',
    'Upper',
    'URDF',
    'URI',
    'UseDynamicMOIRescaling',
    'UseModelFrame',
    'UseParentModelFrame',
    'UsePatchRadius',
    'UseTerrainPaging',
    'Velocity',
    'Vertical',
    'VerticalPosition',
    'VerticalVelocity',
    'ViewController',
    'ViscousFriction',
    'Visual',
    'Visualize',
    'WallTime',
    'Waypoint',
    'Width',
    'Wind',
    'WorldFrameOrientation',
    'World',
    'Wrench',
    'X',
    'XYZ',
    'Y',
    'Z'
]
