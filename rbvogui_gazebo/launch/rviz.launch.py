# Copyright (c) 2022, Robotnik Automation S.L.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

from robotnik_common.launch import RewrittenYaml

# Environment variables
#  USE_SIM_TIME: Use simulation (Gazebo) clock if true
#  NAMESPACE: Namespace of the node stack.
#  ROBOT_ID: Frame id of the robot. (e.g. vectornav_link).
#  RVIZ_CONFIG: Absolute path to the rviz config file.
#  RVIZ_CONFIG_FILE: Name of the rviz config file.

def read_params(ld : launch.LaunchDescription):
    environment = launch.substitutions.LaunchConfiguration('environment')
    namespace = launch.substitutions.LaunchConfiguration('namespace')
    robot_id = launch.substitutions.LaunchConfiguration('robot_id')
    rviz_config = launch.substitutions.LaunchConfiguration('rviz_config')
    rviz_config_file = launch.substitutions.LaunchConfiguration('rviz_config_file')

    # Declare the launch options
    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='environment',
        description='Read params from environment variables.',
        choices=['true', 'false'],
        default_value='true')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='robot_id',
        description='Frame id of the sensor. (e.g. robot).',
        default_value='robot')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='namespace',
        description='Namespace of the nodes.',
        default_value=robot_id)
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='rviz_config_file',
        description='Name of the rviz config file.',
        default_value='default')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='rviz_config',
        description='Absolute path to the rviz config file.',
        default_value=[get_package_share_directory('rbvogui_gazebo'), '/rviz/', rviz_config_file, '.rviz'])
    )
    
    # Parse the launch options
    ret = {}

    if environment == 'false':
        ret = {
        'namespace': namespace,
        'robot_id': robot_id,
        'rviz_config': rviz_config,
        }
    
    else:
        if 'ROBOT_ID' in os.environ:
            ret['robot_id'] = os.environ['ROBOT_ID']
        else: ret['robot_id'] = robot_id

        if 'NAMESPACE' in os.environ:
            ret['namespace'] = os.environ['NAMESPACE']
        elif 'ROBOT_ID' in os.environ:
            ret['namespace'] = os.environ['ROBOT_ID']
        else:  ret['namespace'] = namespace

        if 'RVIZ_CONFIG' in os.environ:
            ret['rviz_config'] = os.environ['RVIZ_CONFIG']
        elif 'RVIZ_CONFIG_FILE' in os.environ:
            ret['rviz_config'] = [get_package_share_directory('rbvogui_gazebo'), 'rviz', os.environ['RVIZ_CONFIG_FILE'], '.rviz']
        else:  ret['rviz_config'] = rviz_config

    return ret


def generate_launch_description():

    ld = launch.LaunchDescription()

    params = read_params(ld)

    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', params['rviz_config'], '-f', [params['robot_id'], '_odom']],
    )

    print(params['rviz_config'])

    ld.add_action(launch_ros.actions.PushRosNamespace(namespace=params['namespace']))
    ld.add_action(rviz2_node)

    return ld
