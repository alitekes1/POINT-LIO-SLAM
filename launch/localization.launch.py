from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile
import os
import yaml

def generate_launch_description():

    point_lio_slam_dir = get_package_share_directory('point-lio-slam')

    localization_config = os.path.join(point_lio_slam_dir, 'config', 'localization_indoor.yaml')


    localization_node = Node(
            package='localization_qn',
            executable='localization_qn_node',
            name='localization_qn_node',
            output='screen',
            parameters=[localization_config],
            remappings=[
                ('/Odometry', '/aft_mapped_to_init'),
                ('/cloud_registered', '/cloud_registered')
            ]
    )

    lioParams = [
        PathJoinSubstitution([
            FindPackageShare('point-lio-slam'),
            'config', 'lio_unilidar_l1.yaml'
        ]),
        {
            'use_imu_as_input': False,  # Change to True to use IMU as input of Point-LIO
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 1,  # Options: 1, 3
            'space_down_sample': True,
            'filter_size_surf': 0.1,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1
            'filter_size_map': 0.1,  # Options: 0.5, 0.3, 0.15, 0.1
            'cube_side_length': 1000.0,  # Option: 1000
            'runtime_pos_log_enable': False,  # Option: True
        }
    ]

    lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=lioParams,
        # prefix='gdb -ex run --args'
    )

    localization_rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_localization',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('point-lio-slam'),
                'rviz',
                'localization_rviz.rviz'
            ])],
            prefix='nice',
            output='screen'
        )
    
    lio_rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_lio',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('point-lio-slam'),
                'rviz',
                'lio_rviz.rviz'
            ])],
            prefix='nice',
            output='screen'
    )


    return LaunchDescription([
        localization_node,
        lio_node,
        localization_rviz_node,
        lio_rviz_node
    ])