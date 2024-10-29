import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import Parameter, ParameterFile
from launch.launch_context import LaunchContext
from launch.utilities import perform_substitutions
from launch.actions import OpaqueFunction

pkgName = 'rv2_server_compositions'
pkgPath = get_package_share_directory(pkgName)


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    params_file_path = perform_substitutions(context, [params_file])

    yaml_dict = yaml.safe_load(open(params_file_path))
    yaml_dict = yaml_dict[[k for k in yaml_dict.keys()][0]]['ros__parameters']

    # Check cli params (string)
    use_sim_time_input = perform_substitutions(context, [use_sim_time])
    if use_sim_time_input:
        yaml_dict['use_sim_time'] = True if use_sim_time_input.lower() == 'true' else False

    cmd_group = GroupAction([
        PushRosNamespace(
            namespace=yaml_dict['namespace']
        ), 

        Node(
            package=pkgName,
            name=yaml_dict['devManageServiceNodeName'] + '_' + yaml_dict['devManageServiceNodeID'] if yaml_dict['devManageServiceNodeName'] and yaml_dict['devManageServiceNodeID'] else None, 
            executable='devmanageserver_server',
            output='screen',
            parameters=[yaml_dict]
        ), 

        Node(
            package=pkgName,
            name=yaml_dict['qosServiceNodeName'] + '_' + yaml_dict['qosServiceNodeID'] if yaml_dict['qosServiceNodeName'] and yaml_dict['qosServiceNodeID'] else None, 
            executable='qosserver_server',
            output='screen',
            parameters=[yaml_dict]
        )

    ])

    return [cmd_group]

def generate_launch_description():
    ###############
    ## ARGUMENTS ##
    ###############
    ld = LaunchDescription([
        # General args

        # use_sim_time will be set under launch_setup().
        DeclareLaunchArgument("use_sim_time", default_value="", description="Use simulation clock if true"), 
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkgPath, 'params', 'params.yaml'), description="Full path to the ROS2 parameters file to use for all launched nodes."), 
        DeclareLaunchArgument("domain_id", default_value="0", description="Set to different value to avoid interference when several computers running ROS2 on the same network."), 
        SetEnvironmentVariable(name='ROS_DOMAIN_ID',value=LaunchConfiguration('domain_id')),
    ])

    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
