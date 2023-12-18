import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ropisim').find('ropisim')
    default_model_path = os.path.join(pkg_share, 'urdf/box_bot.urdf')
    default_box_path = os.path.join(pkg_share, 'model/gazebo_aruco_box/aruco_box/model.sdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav.rviz')
    world_path = os.path.join(pkg_share, 'world/my_world.world')

    gazebo_models_path = os.path.join(pkg_share, 'model', 'gazebo_aruco_box')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path


    # Define launch file arguments
    arg_sim = launch_ros.actions.SetParameter(name='use_sim_time', value=True)
    
    arg_gui = launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
                                        description='Flag to enable joint_state_publisher_gui')
    arg_model = launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Absolute path to robot urdf file')
    arg_rvizconfig = launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file')


    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )


    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'wheel_chair', '-topic', 'robot_description',
        # '-x', '-5.204683', '-y', '-18.104129', '-z', '0.004496',
        '-x', '0', '-y', '0', '-z', '0',
        '-R', '0', '-P', '0', '-Y', '1.722056'],
        output='screen'
    )

    spawn_aruco_box = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'box', '-file', default_box_path,
        # '-x', '-5.204683', '-y', '-18.104129', '-z', '0.004496',
        '-x', '0.0', '-y', '3.0', '-z', '0',
        '-R', '0', '-P', '0', '-Y', '1.722056'],
        output='screen'
    )

    gazebo_exec =  launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen')



  

    #Make launch description
    ld = launch.LaunchDescription()

    #Add launch arguments
    ld.add_action(arg_sim)
    ld.add_action(arg_gui)
    ld.add_action(arg_model)
    ld.add_action(arg_rvizconfig)

    #Add nodes
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity)
    ld.add_action(spawn_aruco_box)
    ld.add_action(gazebo_exec)
    ld.add_action(rviz_node)

    return ld