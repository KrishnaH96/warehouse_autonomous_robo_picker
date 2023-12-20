import os
import launch
from launch import launch_description_sources
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import launch_ros
from launch import actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    


    # Define launch file arguments
    arg_sim = launch_ros.actions.SetParameter(name='use_sim_time', value=True)

    sim_description_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ropisim'),
                'launch',
                'display_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'rvizconfig': PathJoinSubstitution([
                FindPackageShare('ropisim'),
                'rviz',
                'nav.rviz'
            ])
        }.items()
    )
    
    slam_dir = get_package_share_directory('ropislam')

    slam_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                slam_dir + '/launch/slam.launch.py'))
    

    navigation_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ropinav'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': PathJoinSubstitution([
                FindPackageShare('ropinav'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )

    aruco_dir = get_package_share_directory('ros2_aruco')

    aruco_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                aruco_dir + '/launch/aruco_recognition.launch.py'))
  

    #Make launch description
    ld = launch.LaunchDescription()


    #Add nodes
    ld.add_action(arg_sim)
    ld.add_action(sim_description_launch)
    ld.add_action(slam_launch)
    ld.add_action(navigation_launch)
    ld.add_action(aruco_launch)

    return ld