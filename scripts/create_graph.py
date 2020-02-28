#!/usr/bin/python

import rospy
import math
import numpy as np
import copy
import time
from os import path
import pickle
from graham_scan import create_points, graham_scan
import project_utils as pu

"""
    Node class for A* algorithm.
"""
SCALE = 15  # TODO as param
FREE = 0
OCCUPIED = 100
UNKNOWN = -1

ROBOT_SPACE = 1.0  # TODO pick from param (robot radius + inflation radius)
OUT_OF_BOUNDS = -2

INDEX_FOR_X = 1
INDEX_FOR_Y = 0

"""
Graph class for creating and working with graph.
"""


class Graph:

    def __init__(self, environment, robot_count, map_width, map_height, run, termination_metric, robot_id):
        self.latest_map = {}
        self.resolution = 0.05
        self.origin_pose_x = 0
        self.origin_pose_y = 0
        self.map_width = 0
        self.map_height = 0
        self.call_count = 0
        self.environment = environment
        self.robot_count = robot_count
        self.run = run
        self.known_area = 0
        self.known_points = {}
        self.termination_metric = termination_metric
        self.robot_id = robot_id
        self.performance_data = []




    '''
     Pass the message and generate the poses corresponding the explored region
    '''

    def update_graph(self, map_message):
        map_info = map_message.info
        origin_pose = map_info.origin.position
        map_width = map_info.width
        map_height = map_info.height
        self.resolution = map_info.resolution
        self.origin_pose_x = origin_pose.x
        self.origin_pose_y = origin_pose.y
        grid_values = np.array(map_message.data).reshape((map_height, map_width)).astype(np.float32)
        num_rows = grid_values.shape[0]
        num_cols = grid_values.shape[1]
        for row in range(num_rows):
            for col in range(num_cols):
                index = [0] * 2
                index[INDEX_FOR_Y] = num_rows - row - 1
                index[INDEX_FOR_X] = col
                data_val = grid_values[num_rows - row - 1, col]
                self.latest_map[tuple(index)] = data_val
                if data_val == FREE or data_val == OCCUPIED:
                    self.known_points[tuple(index)] = None
        self.known_area = len(self.known_points)

    def pixel2pose(self, point, origin_x, origin_y, resolution):
        new_p = [0] * 2
        new_p[INDEX_FOR_Y] = round(origin_x + point[INDEX_FOR_X] * resolution, 0)
        new_p[INDEX_FOR_X] = round(origin_y + point[INDEX_FOR_Y] * resolution, 0)
        return tuple(new_p)

    def pose2pixel(self, pose, origin_x, origin_y, resolution):
        x = round((pose[INDEX_FOR_X] - origin_y) / resolution)
        y = round((pose[INDEX_FOR_Y] - origin_x) / resolution)
        position = [0] * 2
        position[INDEX_FOR_X] = x
        position[INDEX_FOR_Y] = y
        return tuple(position)

    def get_frontiers(self, count=10):
        frontiers = []
        start_time = time.time()
        all_points = list(self.latest_map)
        free_points = [p for p in all_points if self.latest_map[p] == FREE]
        if free_points:
            hull, boundary_points = graham_scan(free_points, count, False)
            new_information, known_points, unknown_points = self.compute_new_information(boundary_points)
            while len(new_information) > 0:
                best_point = max(new_information, key=new_information.get)
                new_p = self.pixel2pose(best_point, self.origin_pose_x, self.origin_pose_y, self.resolution)
                frontiers.append(new_p)
                del new_information[best_point]
                if len(frontiers) == count:
                    break
            now = time.time()
            t = (now - start_time)
            self.performance_data.append({'time': now, 'type': 2, 'robot_id': 0, 'computational_time': t})
            # self.save_data([], "recurrent/performance_{}_{}_{}.pickle".format(self.environment, self.robot_count, self.run))
        else:
            rospy.logerr("No free points available")
        return frontiers

    '''
     Get the points which are closest to the unknown area
    '''

    def get_rendezvous_points(self, origin, range_val, count=10):
        rendezvous_points = []
        start = time.time()
        if self.latest_map:
            all_points = list(self.latest_map)
            free_points = [p for p in all_points if self.latest_map[p] == FREE and self.D(origin, self.pixel2pose(p,
                                                                                                                  self.origin_pose_x,
                                                                                                                  self.origin_pose_y,
                                                                                                                  self.resolution)) <= range_val]
            if free_points:
                hull, boundary_points = graham_scan(free_points, count, False)
                for b in boundary_points:
                    rendezvous_points.append(
                        self.pixel2pose(b, self.origin_pose_x, self.origin_pose_y, self.resolution))
                now = time.time()
                t = now - start
                self.performance_data.append({'time': now, 'type': 1, 'robot_id': 0, 'computational_time': t})
            else:
                rospy.logerr("No free points available")
        else:
            rospy.logerr("No map")
        return rendezvous_points

    def D(self, p, q):
        dx = q[INDEX_FOR_X] - p[INDEX_FOR_X]
        dy = q[INDEX_FOR_Y] - p[INDEX_FOR_Y]
        return math.sqrt(dx ** 2 + dy ** 2)

    def area(self, point, orientation):
        known_points = []
        unknown_points = []
        for d in np.arange(0, 40.0, 1):
            distance_points = []
            for theta in range(-1 * 360 // 2, 360 + 1):
                angle = np.deg2rad(theta) + orientation
                x = point[INDEX_FOR_X] + d * np.cos(angle)
                y = point[INDEX_FOR_Y] + d * np.sin(angle)
                new_p = [0] * 2
                new_p[INDEX_FOR_X] = x
                new_p[INDEX_FOR_Y] = y
                new_p = tuple(new_p)
                distance_points.append(new_p)
            unique_points = list(set(distance_points))
            for p in unique_points:
                if self.is_free(p) or self.is_obstacle(p):
                    known_points.append(p)
                else:
                    unknown_points.append(p)
        return known_points, unknown_points

    def is_obstacle(self, p):
        xc = np.round(p[INDEX_FOR_X])
        yc = np.round(p[INDEX_FOR_Y])
        new_p = [0] * 2
        new_p[INDEX_FOR_X] = xc
        new_p[INDEX_FOR_Y] = yc
        new_p = tuple(new_p)
        return new_p in self.latest_map and self.latest_map[new_p] == OCCUPIED

    def is_free(self, p):
        xc = np.round(p[INDEX_FOR_X])
        yc = np.round(p[INDEX_FOR_Y])
        new_p = [0] * 2
        new_p[INDEX_FOR_X] = xc
        new_p[INDEX_FOR_Y] = yc
        new_p = tuple(new_p)
        return new_p in self.latest_map and self.latest_map[new_p] == FREE

    def compute_new_information(self, selected_points):
        resolution = 1
        new_information = {}
        known_points = {}
        unknown_points = {}
        for point in selected_points:
            ks, us = self.area(point, 0)
            unknown_area = len(us) * resolution ** 2
            if point not in new_information:
                new_information[point] = unknown_area
            known_points[point] = ks
            unknown_points[point] = us
        return new_information, known_points, unknown_points

    def get_optimal_points(self, origin, points):
        point_dict = {}
        for point in points:
            distance = math.sqrt((origin[0] - point[0]) ** 2 + (origin[1] - point[1]) ** 2)
            if distance > 2 * ROBOT_SPACE:
                point_dict[point] = distance
        optimal_points = [key for key, value in sorted(point_dict.items(), key=lambda item: item[1])]
        return optimal_points

    def save_all_data(self):
        pu.save_data(self.performance_data,
                     "recurrent/performance_{}_{}_{}_{}_{}.pickle".format(self.environment,
                                                                                       self.robot_count, self.run,
                                                                                       self.termination_metric,
                                                                                       self.robot_id))
        rospy.signal_shutdown('Graph: Shutdown command received!')


"""
    Function for starting node and creating graph. 
"""
if __name__ == "__main__":
    # rospy.init_node("create_graph")
    graph = Graph()
