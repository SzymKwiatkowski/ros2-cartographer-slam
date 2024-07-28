# Copyright 2023 zymon Kwiatkowski
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

pkg_prefix = FindPackageShare('cartographer_demo')

def launch_setup(context, *args, **kwargs):
    cartographer_pkg_name = 'cartographer_ros'
    
    config_file = 'localization.lua'
    map_path = LaunchConfiguration('map_path').perform(context)
    mapping = IfCondition(LaunchConfiguration('mapping')).evaluate(context)
    state_filename = map_path
        
    if mapping:
        config_file = 'mapping.lua'
        state_filename = ''
    
    cartographer_ros_node = Node(
       package = cartographer_pkg_name,
       executable = 'cartographer_node',
       parameters = [{
           'use_sim_time': True,
       }],
       arguments = [
            '-configuration_directory', PathJoinSubstitution(
                [pkg_prefix, 'config']),
            '-configuration_basename', config_file,
            '-load_state_filename', state_filename
        ],
       remappings=[
            ('imu', '/sensing/vesc/imu'),
            ('odom', '/localization/kinematic_state'),
            ('scan', '/sensing/lidar/scan')
        ]
    )
    
    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05}],
        )
    
    return [
        cartographer_ros_node,
        cartographer_occupancy_grid_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    # Localization
    add_launch_arg('mapping', 'false')
    add_launch_arg('map_path', PathJoinSubstitution([pkg_prefix, 'config', 'result.pbstream']))

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
