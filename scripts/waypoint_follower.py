#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory

import json
import os

# Obtener la ruta correcta del paquete
package_name = 'vineyard-mybot'
package_path = get_package_share_directory(package_name)
waypoints_file = os.path.join(package_path, 'config', 'waypoints_nav2.json')

with open(waypoints_file, 'r') as json_file:
    navigation_data = json.load(json_file)

print(navigation_data[0]['x'])

def main() -> None:
    rclpy.init()

    navigator = BasicNavigator()

    # NO establecer pose inicial automáticamente - se debe hacer manualmente en RViz
    # usando "2D Pose Estimate" para localizar correctamente el robot
    
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = navigation_data[0]['x']
    # initial_pose.pose.position.y = navigation_data[0]['y']
    # initial_pose.pose.orientation.z = navigation_data[0]['orientation_z']
    # initial_pose.pose.orientation.w = navigation_data[0]['orientation_w']
    # navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()
    
    # Esperar a que el usuario establezca la pose inicial en RViz
    print("IMPORTANTE: Antes de continuar, establece la pose inicial del robot en RViz usando '2D Pose Estimate'")
    print("Presiona Enter cuando hayas establecido la pose inicial...")
    input()

    # If desired, you can change or load the map as well
    # navigator.changeMap(map_file)  # Comentado porque el mapa ya está cargado por Nav2

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    goal_poses = []

    for pose in navigation_data:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose['x']
        goal_pose.pose.position.y = pose['y']
        goal_pose.pose.orientation.w = pose['orientation_w']
        goal_pose.pose.orientation.z = pose['orientation_z']
        goal_poses.append(goal_pose)

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose1)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(goal_poses))
            )
            now = navigator.get_clock().now()

            # Timeout más largo para dar tiempo al robot (10 minutos)
            if now - nav_start > Duration(seconds=1200.0):
                navigator.cancelTask()
                print("Timeout alcanzado - cancelando navegación")

            # Comentamos la preemption que cambia el objetivo
            # if now - nav_start > Duration(seconds=35.0):
            #     goal_pose4 = PoseStamped()
            #     goal_pose4.header.frame_id = 'map'
            #     goal_pose4.header.stamp = now.to_msg()
            #     goal_pose4.pose.position.x = 0.0
            #     goal_pose4.pose.position.y = 0.0
            #     goal_pose4.pose.orientation.w = 1.0
            #     goal_pose4.pose.orientation.z = 0.0
            #     goal_poses = [goal_pose4]
            #     nav_start = now
            #     follow_waypoints_task = navigator.followWaypoints(goal_poses)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        (error_code, error_msg) = navigator.getTaskError()
        print('Goal failed!{error_code}:{error_msg}')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()