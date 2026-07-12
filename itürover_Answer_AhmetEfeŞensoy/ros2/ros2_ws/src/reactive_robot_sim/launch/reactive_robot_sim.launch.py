#!/usr/bin/env python3
"""
reactive_robot_sim.launch.py — ITU Rover engel kaçınma simülasyon ortamı (ROS 2).

ROS 1 reactive_robot_sim.launch dosyasının ROS 2 (Gazebo Classic) karşılığı.
Gazebo'yu obs_corridor_world dünyasıyla başlatır, robot_state_publisher ile
robot_description'ı yayınlar, robotu spawn eder ve isteğe bağlı RViz'i açar.

Argümanlar:
  gui           : Gazebo istemci (client) GUI'si açık mı (varsayılan true)
  rviz          : RViz2 açık mı (varsayılan false)
  model         : {burger_rgbd, burger, waffle, waffle_pi}
  ground_truth  : p3d ground-truth odometry plugin'i (varsayılan false)
  world         : dünya adı (varsayılan obs_corridor_world)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('reactive_robot_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ── Argümanlar ──────────────────────────────────────────────────────────
    gui          = LaunchConfiguration('gui')
    rviz         = LaunchConfiguration('rviz')
    model        = LaunchConfiguration('model')
    ground_truth = LaunchConfiguration('ground_truth')
    world        = LaunchConfiguration('world')

    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false')
    declare_model = DeclareLaunchArgument(
        'model', default_value='burger_rgbd',
        description='{burger_rgbd, burger, waffle, waffle_pi}')
    declare_ground_truth = DeclareLaunchArgument('ground_truth', default_value='false')
    declare_world = DeclareLaunchArgument('world', default_value='obs_corridor_world')

    # Spawn pozu — ROS 1 launch'taki obs_corridor_world spawn_args ile aynı:
    #   -x 0 -y 9.5 -z 0 -R 0 -P 0 -Y -1.57
    spawn_x   = '-0.0'
    spawn_y   = '9.5'
    spawn_z   = '0.0'
    spawn_yaw = '-1.57'

    # Gazebo model yollarını export et (model:// URI'leri için).
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            os.path.join(pkg_share, 'models'),
            os.pathsep,
            os.environ.get('GAZEBO_MODEL_PATH', ''),
        ],
    )

    # ── robot_description (xacro) ───────────────────────────────────────────
    # $(find ...) yerine ROS 2 xacro + paylaşılan dizin yolu kullanılır.
    # model bir LaunchConfiguration olduğundan yol parçaları liste olarak
    # birleştirilir; xacro çağrısı runtime'da tek komuta çözülür.
    xacro_path = os.path.join(pkg_share, 'urdf')
    robot_description_content = Command([
        'xacro ', xacro_path, '/turtlebot3_', model, '.urdf.xacro',
        ' ground_truth:=', ground_truth,
    ])

    # ── Gazebo (Classic) sunucu + istemci ───────────────────────────────────
    world_file = [os.path.join(pkg_share, 'worlds'), '/', world, '.world']

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'pause': 'false',
            'verbose': 'false',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui),
    )

    # ── robot_state_publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )

    # ── Robotu spawn et ─────────────────────────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_urdf',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', ['turtlebot3_', model],
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
    )

    # ── RViz2 (isteğe bağlı) ────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'reactive_robot.rviz')],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_gui,
        declare_rviz,
        declare_model,
        declare_ground_truth,
        declare_world,
        set_model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        rviz_node,
    ])
