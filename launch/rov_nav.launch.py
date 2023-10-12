

"""
Launch a simple nav using mocap system.

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    multiorca_dir = get_package_share_directory('multiple_orca')      

    # need to source the mocap4ros2 workspace
    # mocap 

    # mavros

    # rviz

    # nav2

    # static tf



    # define variables that are not depending on the namespace
    orca_params_file = os.path.join(multiorca_dir, 'params', 'sim_orca_params.yaml')

    # to bring up a rov, there should exit configuration files for the rov.
    # cfg/{rov-name}_bridge.yaml
    # models/{rov-name}
    # params/sim_mavros_params_{rov-name}
    robots = [
        {'name': 'bluerov2_heavy', 'x_pose': 0.0, 'y_pose': 0.0, 'z_pose': 0.0,
                           'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0, 'instance':'-I0',
                           'home':'33.810313,-118.39386700000001,0.0,270.0',
                           'sysid':'1',
                        #    'fcu_url':'tcp://localhost:5760',
                        #    'gcs_url':'udp://@localhost:14550'
                           },
        # {'name': 'tur2', 'x_pose': 0.0, 'y_pose': 3.0, 'z_pose': 0.0,
        #                    'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0, 'instance':'-I1',
        #                    'home':'33.810311,-118.39386700000001,0.0,270.0',
        #                    'sysid':'2',
        #                 #    'fcu_url':'tcp://localhost:5770',
        #                 #    'gcs_url':'udp://@localhost:14560'
        #                    },
        # {'name': 'tur3', 'x_pose': 1.5, 'y_pose': 1.5, 'z_pose': 0.0,
        #                    'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0, 'instance':'-I2',
        #                    'home':'33.810311,-118.39386700000001,0.0,270.0',
        #                    'sysid':'3',
        #                 #    'fcu_url':'tcp://localhost:5780',
        #                 #    'gcs_url':'udp://@localhost:14570'
        #                    },
        ]
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')] 
    
    use_sim_time = LaunchConfiguration('use_sim_time')   

    
    # to use the gazebo time, /clock topic should be published. You can do it using parameter bride.
    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='use_sim_time param')
    
    
    instances_cmds = []
    for robot in robots:        
        rov_ns = robot['name']

        mavros_params_file = os.path.join(multiorca_dir, 'params', f"sim_mavros_params_{robot['name']}.yaml")

        rviz_config_file = LaunchConfiguration('rviz_config')   

        declare_rviz_config_file_cmd = DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(multiorca_dir, 'cfg', 'nav2_namespaced_view.rviz'),
            description='Full path to the RVIZ config file to use.')
        
        declare_arg_base_cmd = DeclareLaunchArgument(
            'base',
            default_value='True',
            description='Launch base controller?',
        )

        declare_arg_mavros_cmd = DeclareLaunchArgument(
            'mavros',
            default_value='True',
            description='Launch mavros?',
        )

        declare_arg_nav_cmd = DeclareLaunchArgument(
            'nav',
            default_value='True',
            description='Launch navigation?',
        )

        declare_arg_slam_cmd = DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Launch SLAM?',
        )   

        declare_rviz_cmd = DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Launch rviz?',
        )     

        # currently world frames are being published in each rov namespace. 
        # TODO need to make it globally available
        tf_static_pub_cmd = ComposableNodeContainer(
            name='tf_container_1',
            package='rclcpp_components',
            executable='component_container',  
            namespace=rov_ns,          
            composable_node_descriptions=[
                ComposableNode(
                    package='tf2_ros',
                    plugin='tf2_ros::StaticTransformBroadcasterNode',
                    name='static_tf_world2map',
                    namespace=rov_ns,
                    parameters=[{
                        'frame_id':'world',
                        'child_frame_id':'map',
                        'translation.x': robot['x_pose'],
                        'translation.y': robot['y_pose'],
                        'translation.z': robot['z_pose'],
                        'rotation.x': robot['x'],
                        'rotation.y': robot['y'],
                        'rotation.z': robot['z'],
                        'rotation.w': robot['w']
                        },
                        {'use_sim_time':use_sim_time}],
                    remappings=remappings)
            ],
            output='screen'
        )

        start_rviz_cmd = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(multiorca_dir,'launch', 'rviz_launch.py')),
                    launch_arguments={'namespace': TextSubstitution(text=rov_ns),
                                    'use_namespace': 'True',
                                    'rviz_config': rviz_config_file,
                                    'use_sim_time':use_sim_time}.items(),
                    condition=IfCondition(LaunchConfiguration('rviz')))
        

        # Bring up Orca and Nav2 nodes
        orca_bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(multiorca_dir, 'launch', 'bringup.py')),
            launch_arguments={
                'namespace':rov_ns,
                # 'namespace':LaunchConfiguration('namespace'),
                'base': LaunchConfiguration('base'),
                'mavros_ns':f"{rov_ns}/mavros",
                'mavros': LaunchConfiguration('mavros'),
                'mavros_params_file': mavros_params_file,
                'nav': LaunchConfiguration('nav'),
                'orca_params_file': orca_params_file,
                'slam': LaunchConfiguration('slam'),
                'use_sim_time':use_sim_time
            }.items(),
        )      

        pose_pub_cmd =  Node(
            package='multiple_orca',
            executable='tf2pose_publisher',
            output='screen',
            namespace=rov_ns,
            parameters=[{
                'target_tf':'bluerov2_heavy',
                'fixed_tf':'map',        
            }],
            remappings=[('/tf', 'tf'),
                  ('/tf_static', 'tf_static')] 
        )


        instances_cmds.append(declare_rviz_config_file_cmd)
        instances_cmds.append(declare_arg_base_cmd)
        instances_cmds.append(declare_arg_mavros_cmd)
        instances_cmds.append(declare_arg_nav_cmd)
        instances_cmds.append(declare_arg_slam_cmd)   
        instances_cmds.append(declare_rviz_cmd)                
        instances_cmds.append(start_rviz_cmd)
        instances_cmds.append(orca_bringup_cmd)
        instances_cmds.append(tf_static_pub_cmd)
        instances_cmds.append(pose_pub_cmd)

    
    ld = LaunchDescription()
        
    ld.add_action(declare_use_sim_time_cmd)

    for inst in instances_cmds:
        ld.add_action(inst)
    
    return ld


