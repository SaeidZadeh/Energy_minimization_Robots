#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def gen_robot_list(number_of_robots):
    col_names=["purple", "red", "blue", "green", "yellow", "orange", "brown", "pink", "gold"]
    col_list=["0.5 0.1 0.61 1.0", "0.9 0.0 0.0 1.0", "0.2 0.2 0.3 1.0", "0.0 0.8 0.0 1.0", "1.0 1.0 0.1 1.0", "1.0 0.4 0.1 1.0", "0.9 0.8 0.8 1.0", "1.0 0.5 0.8 1.0", "0.9 0.7 0.1 1.0"]
    mesh_files = ["package://box_bot_description/meshes/robot0.dae",
                  "package://box_bot_description/meshes/robot1.dae",
                  "package://box_bot_description/meshes/robot2.dae",
                  "package://box_bot_description/meshes/robot3.dae",
                  "package://box_bot_description/meshes/robot4.dae",
                  "package://box_bot_description/meshes/robot5.dae",
                  "package://box_bot_description/meshes/robot6.dae",
                  "package://box_bot_description/meshes/robot7.dae",
                  "package://box_bot_description/meshes/robot8.dae"]
    robots = []

    for i in range(number_of_robots):
        robot_name = "box_bot"+str(i)
        y_pos = float(i)-10.5
        robots.append({'name': robot_name, 'x_pose': -11.5, 'y_pose': y_pos, 'z_pose': 0.01, 'col_name': col_names[i], 'color': col_list[i], 'mesh_file':mesh_files[i]})


    return robots 

def generate_launch_description():

    urdf = os.path.join(get_package_share_directory('box_bot_description'), 'robot/', 'box_bot_v2.urdf')
    pkg_box_bot_description = get_package_share_directory('box_bot_description')
    assert os.path.exists(urdf), "Thebox_bot.urdf doesnt exist in "+str(urdf)

    # Names and poses of the robots
    robots = gen_robot_list(9)

    # We create the list of spawn robots commands
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_box_bot_description, 'launch',
                                                           'spawn_box_bot_launch.py')),
                launch_arguments={
                                  'robot_urdf': urdf,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'robot_name': robot['name'],
                                  'robot_namespace': robot['name'],
                                  'col_name': robot["col_name"],
                                  'color': robot["color"],
                                  'mesh_file':robot['mesh_file']
                                  }.items()))

    # Create the launch description and populate
    ld = LaunchDescription()
    
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld