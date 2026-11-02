"""Full NOVA pipeline: test sim, perception, UKF, C++ NMPC and, in acro mode, the converter.

  ros2 launch launch/nova.launch.py duration:=20.0 acro_mode:=true viewer:=false
"""
import os
import shutil
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'sim'))
from scenario import make_trajectory, spawn_offset  # noqa: E402


def generate_launch_description():
    acro_mode = LaunchConfiguration('acro_mode')
    detection_view = LaunchConfiguration('detection_view')
    init_position = [float(v) for v in spawn_offset(make_trajectory())]
    sim_time = {'use_sim_time': True}

    true_state = LaunchConfiguration('true_state')
    ukf = Node(package='target_ukf_ros', executable='ukf_node', output='screen',
               condition=UnlessCondition(true_state), parameters=[{
        **sim_time, 'init_position': init_position, 'pos_noise_std': [0.30, 0.30, 0.30],
        'use_target_accel': True, 'publish_rate': 100.0}])
    perception = Node(package='target_perception_ros', executable='perception_node',
                      output='screen', parameters=[sim_time])
    nmpc = Node(package='quad_nmpc_ros_cpp', executable='nmpc_node', output='screen',
                remappings=[('/odom', '/ukf/odom')], parameters=[{
                    **sim_time, 'mass': 1.25, 'k_tau': 20.0, 'gravity': 9.81,
                    'reference_topic': '/nmpc/reference', 'target_accel_topic': '/target/accel',
                    'log_file': LaunchConfiguration('nmpc_log')}])
    converter = Node(package='quad_nmpc_ros_cpp', executable='trpy_converter_node', output='screen',
                     condition=IfCondition(acro_mode), parameters=[{**sim_time, 'k_tau': 20.0}])
    sim = ExecuteProcess(
        # ros2 launch itself runs under system Python; the sim needs the env that owns mujoco
        cmd=[shutil.which('python'), os.path.join(ROOT, 'sim', 'nova_sim_node.py'), '--ros-args',
             '-p', ['acro_mode:=', acro_mode],
             '-p', ['duration:=', LaunchConfiguration('duration')],
             '-p', ['viewer:=', LaunchConfiguration('viewer')],
             '-p', ['seed:=', LaunchConfiguration('seed')],
             '-p', ['true_state:=', true_state],
             '-p', ['target_speed:=', LaunchConfiguration('target_speed')],
             '-p', ['image_subscribers:=',
                    PythonExpression(["2 if '", detection_view, "' == 'true' else 1"])]],
        output='screen', additional_env={'MUJOCO_GL': 'egl'})

    view = ExecuteProcess(
        cmd=[shutil.which('python'), os.path.join(ROOT, 'sim', 'detection_view_node.py')],
        output='screen', condition=IfCondition(detection_view))

    recorder = ExecuteProcess(
        cmd=[shutil.which('python'), os.path.join(ROOT, 'test', 'record_node.py'), '--ros-args',
             '-p', ['duration:=', LaunchConfiguration('duration')]],
        output='screen', condition=IfCondition(LaunchConfiguration('record')), sigterm_timeout='30')

    return LaunchDescription([
        DeclareLaunchArgument('acro_mode', default_value='true'),
        DeclareLaunchArgument('duration', default_value='20.0'),
        DeclareLaunchArgument('viewer', default_value='false'),
        DeclareLaunchArgument('detection_view', default_value='false'),
        DeclareLaunchArgument('true_state', default_value='false'),
        DeclareLaunchArgument('target_speed', default_value='3.0'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('nmpc_log', default_value='/tmp/nova_nmpc_log.csv'),
        recorder, ukf, perception, nmpc, converter, view, sim,
        RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[EmitEvent(event=Shutdown())])),
    ])
