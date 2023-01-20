# Copyright 2019 Intelligent Robotics Lab
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The launcher contain the information to create the problem istance and the
# information to activate the different nodes
def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_task5')
    
    # Create a namespace
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    # Create a launch description 
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        # Define where the model file is located and which namespace to use
        launch_arguments={
          'model_file': example_dir + '/pddl/task5_domain.pddl',
          'namespace': namespace
          }.items()) 
                
    
    move_unloaded_robot_cmd = Node(
        package='plansys2_task5',
        executable='move_loaded_robot_action_node',
        name='move_loaded_robot_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 

    move_loaded_robot_cmd = Node(
        package='plansys2_task5',
        executable='move_unloaded_robot_action_node',
        name='move_unloaded_robot_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 


    load_robot_cmd = Node(
        package='plansys2_task5',
        executable='load_robot_action_node',
        name='load_robot_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 

    unload_robot_cmd = Node(
        package='plansys2_task5',
        executable='unload_robot_action_node',
        name='unload_robot_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 


    fill_box_with_supply_charge_cmd = Node(
        package='plansys2_task5',
        executable='fill_box_with_supply_action_node',
        name='fill_box_with_supply_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])   

    unfill_box_with_supply_charge_cmd = Node(
        package='plansys2_task5',
        executable='unfill_box_with_supply_action_node',
        name='unfill_box_with_supply_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])   


    move_1_loaded_carrier_action_node_cmd = Node(
        package='plansys2_task5',
        executable='move_1_loaded_carrier_action_node',
        name='move_1_loaded_carrier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 

    move_1_2_loaded_carrier_action_node_cmd = Node(
        package='plansys2_task5',
        executable='move_1_2_loaded_carrier_action_node',
        name='move_1_2_loaded_carrier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 

    move_1_2_3_loaded_carrier_action_node_cmd = Node(
        package='plansys2_task5',
        executable='move_1_2_3_loaded_carrier_action_node',
        name='move_1_2_3_loaded_carrier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 

    move_1_2_3_4_loaded_carrier_action_node_cmd = Node(
        package='plansys2_task5',
        executable='move_1_2_3_4_loaded_carrier_action_node',
        name='move_1_2_3_4_loaded_carrier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 


    load_carrier_position_1_action_node_cmd = Node(
        package='plansys2_task5',
        executable='load_carrier_position_1_action_node',
        name='load_carrier_position_1_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 

    load_carrier_position_2_action_node_cmd = Node(
        package='plansys2_task5',
        executable='load_carrier_position_2_action_node',
        name='load_carrier_position_2_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 

    load_carrier_position_3_action_node_cmd = Node(
        package='plansys2_task5',
        executable='load_carrier_position_3_action_node',
        name='load_carrier_position_3_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 

    load_carrier_position_4_action_node_cmd = Node(
        package='plansys2_task5',
        executable='load_carrier_position_4_action_node',
        name='load_carrier_position_4_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 


    unload_carrier_position_1_action_node_cmd = Node(
        package='plansys2_task5',
        executable='unload_carrier_position_1_action_node',
        name='unload_carrier_position_1_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
        
    unload_carrier_position_2_action_node_cmd = Node(
        package='plansys2_task5',
        executable='unload_carrier_position_2_action_node',
        name='unload_carrier_position_2_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 

    unload_carrier_position_3_action_node_cmd = Node(
        package='plansys2_task5',
        executable='unload_carrier_position_3_action_node',
        name='unload_carrier_position_3_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
        
    unload_carrier_position_4_action_node_cmd = Node(
        package='plansys2_task5',
        executable='unload_carrier_position_4_action_node',
        name='unload_carrier_position_4_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 


    # Create an istance of the launch description and populate it
    ld = LaunchDescription()

    # Add the declare namespace defined above
    ld.add_action(declare_namespace_cmd)

    # Add the launch options defined above
    ld.add_action(plansys2_cmd)

    ld.add_action(move_unloaded_robot_cmd)
    ld.add_action(move_loaded_robot_cmd)

    ld.add_action(load_robot_cmd)
    ld.add_action(unload_robot_cmd)

    ld.add_action(fill_box_with_supply_charge_cmd)
    ld.add_action(unfill_box_with_supply_charge_cmd)

    ld.add_action(move_1_loaded_carrier_action_node_cmd)
    ld.add_action(move_1_2_loaded_carrier_action_node_cmd)
    ld.add_action(move_1_2_3_loaded_carrier_action_node_cmd)
    ld.add_action(move_1_2_3_4_loaded_carrier_action_node_cmd)

    ld.add_action(load_carrier_position_1_action_node_cmd)
    ld.add_action(load_carrier_position_2_action_node_cmd)
    ld.add_action(load_carrier_position_3_action_node_cmd)
    ld.add_action(load_carrier_position_4_action_node_cmd)

    ld.add_action(unload_carrier_position_1_action_node_cmd)
    ld.add_action(unload_carrier_position_2_action_node_cmd)
    ld.add_action(unload_carrier_position_3_action_node_cmd)
    ld.add_action(unload_carrier_position_4_action_node_cmd)

    # Return the launch descriptor
    return ld
