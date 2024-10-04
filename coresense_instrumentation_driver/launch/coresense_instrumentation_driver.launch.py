# Copyright 2023 Intelligent Robotics Lab
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
import yaml
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('coresense_instrumentation_driver'), 
        'config',
        'system.yaml'
    )
    
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    
    composable_nodes = []
    ns = ''

    for node in config['nodes']:
        name = node['name']
        topic = node['topic']
        msg = node['msg']
        node_type = node['node_type']

        node_params = {
            'topic': topic,
            'topic_type': msg,
            'type': node_type
        }

        qos_profile = node.get('qos_profile', {})
        
        if 'qos_history' in qos_profile:
            node_params['qos_history'] = qos_profile['qos_history']
        
        if 'qos_queue' in qos_profile:
            node_params['qos_queue'] = qos_profile['qos_queue']

        if 'qos_reliability' in qos_profile:
            node_params['qos_reliability'] = qos_profile['qos_reliability']
        
        if 'qos_durability' in qos_profile:
            node_params['qos_durability'] = qos_profile['qos_durability']

        if 'topic_name' in node:
            node_params['topic_name'] = node['topic_name']

        composable_node = ComposableNode(
            package='coresense_instrumentation_driver',
            plugin='coresense_instrumentation_driver::Instrumentation'
                    + node_type + '<' + msg + '>',
            name=name + '_node',
            namespace=ns,
            parameters=[node_params], 
        )

        composable_nodes.append(composable_node)

    container = ComposableNodeContainer(
        name='coresense_container',
        namespace=ns,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(container)

    return ld

