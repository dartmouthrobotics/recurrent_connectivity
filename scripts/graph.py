#!/usr/bin/python
import copy
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import math
import numpy as np
from scipy.spatial import Voronoi
import time
import rospy
from threading import Lock
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from gvgexploration.msg import *
from gvgexploration.srv import *
import project_utils as pu
import tf
from graham_scan import graham_scan
from time import sleep
import shapely.geometry as sg
from nav_msgs.msg import *
from nav2d_navigator.msg import *
from std_srvs.srv import *
from project_utils import INDEX_FOR_X, INDEX_FOR_Y, save_data
from std_msgs.msg import String
from numpy.linalg import norm
import shapely.geometry as sg
from shapely.geometry.polygon import Polygon
from scipy.ndimage import minimum_filter
from tf import TransformListener
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose, PointStamped

INF = 100000
SCALE = 10
FREE = 0.0
OCCUPIED = 100.0
UNKNOWN = -1.0
SMALL = 0.000000000001
FONT_SIZE = 16
MARKER_SIZE = 12


class Grid:
    """Occupancy Grid."""

    def __init__(self, map_msg):
        self.header = map_msg.header
        self.origin_translation = [map_msg.info.origin.position.x,
                                   map_msg.info.origin.position.y, map_msg.info.origin.position.z]
        self.origin_quaternion = [map_msg.info.origin.orientation.x,
                                  map_msg.info.origin.orientation.y,
                                  map_msg.info.origin.orientation.z,
                                  map_msg.info.origin.orientation.w]
        self.grid = np.reshape(map_msg.data,
                               (map_msg.info.height,
                                map_msg.info.width))  # shape: 0: height, 1: width.
        self.resolution = map_msg.info.resolution  # cell size in meters.

        self.tf_listener = TransformListener()  # Transformation listener.

        self.transformation_matrix_map_grid = self.tf_listener.fromTranslationRotation(
            self.origin_translation,
            self.origin_quaternion)

        self.transformation_matrix_grid_map = np.linalg.inv(self.transformation_matrix_map_grid)

        # Array to check neighbors.
        self.cell_radius = int(10 / self.resolution)  # TODO parameter
        self.footprint = np.ones((self.cell_radius + 1, self.cell_radius + 1))
        # self.plot()

    def plot(self):
        fig1 = plt.gcf()
        plt.imshow(self.grid)
        plt.colorbar()
        plt.gca().invert_yaxis()
        plt.xlabel('xlabel', fontsize=18)
        plt.ylabel('ylabel', fontsize=16)
        fig1.savefig('map.png', dpi=100)

    def cell_at(self, x, y):
        """Return cell value at x (column), y (row)."""
        return self.grid[int(y), int(x)]
        # return self.grid[x + y * self.width]

    def is_free(self, x, y):
        if self.within_boundaries(x, y):
            return 0 <= self.cell_at(x, y) < 50  # TODO: set values.
        else:
            return False

    def is_obstacle(self, x, y):
        if self.within_boundaries(x, y):
            return self.cell_at(x, y) >= 50  # TODO: set values.
        else:
            return False

    def is_unknown(self, x, y):
        if self.within_boundaries(x, y):
            return self.cell_at(x, y) < 0  # TODO: set values.
        else:
            return True

    def within_boundaries(self, x, y):
        if 0 <= y < self.grid.shape[0] and 0 <= x < self.grid.shape[1]:
            return True
        else:
            return False

    def convert_coordinates_i_to_xy(self, i):
        """Convert coordinates if the index is given on the flattened array."""
        x = i % self.grid.shape[1]  # col
        y = i / self.grid.shape[1]  # row
        return x, y

    def wall_cells(self):
        """
        Return only *wall cells* -- i.e. obstacle cells that have free or
            unknown cells as neighbors -- as columns and rows.
        """

        # Array to check neighbors.
        window = np.asarray([
            [1, 1, 1],
            [1, 0, 1],
            [1, 1, 1]
        ])
        # Return all cells that are obstacles and satisfy the neighbor_condition
        neighbor_condition = self.grid > minimum_filter(self.grid,
                                                        footprint=window, mode='constant', cval=10000)  # TODO: value.
        obstacles = np.nonzero((self.grid >= OCCUPIED) & neighbor_condition)
        obstacles = np.stack((obstacles[1], obstacles[0]),
                             axis=1)  # 1st column x, 2nd column, y.

        return obstacles

    def is_frontier(self, previous_cell, current_cell,
                    distance):
        """current_cell a frontier?"""
        v = previous_cell - current_cell
        u = v / np.linalg.norm(v)

        end_cell = current_cell - distance * u

        x1, y1 = current_cell.astype(int)
        x2, y2 = end_cell.astype(int)

        for p in list(bresenham(x1, y1, x2, y2)):
            if self.is_unknown(*p):
                return True
            elif self.is_obstacle(*p):
                return False
        return False

    def line_in_unknown(self, start_cell, end_cell):
        x1, y1 = start_cell.astype(int)
        x2, y2 = end_cell.astype(int)

        unknown_cells_counter = 0
        for p in list(bresenham(x1, y1, x2, y2)):
            if self.is_unknown(*p):
                unknown_cells_counter += 1
            elif self.is_obstacle(*p):
                return False
        return unknown_cells_counter

    def unknown_area_approximate(self, cell):
        """Approximate unknown area with the robot at cell."""
        cell_x = int(cell[INDEX_FOR_X])
        cell_y = int(cell[INDEX_FOR_Y])

        min_x = np.max((0, cell_x - self.cell_radius))
        max_x = np.min((self.grid.shape[1], cell_x + self.cell_radius + 1))
        min_y = np.max((0, cell_y - self.cell_radius))
        max_y = np.min((self.grid.shape[0], cell_y + self.cell_radius + 1))

        return (self.grid[min_y:max_y, min_x:max_x] < FREE).sum()

    def unknown_area(self, cell):  # TODO orientation of the robot if fov is not 360 degrees
        """Return unknown area with the robot at cell"""
        unknown_cells = set()

        shadow_angle = set()
        cell_x = int(cell[INDEX_FOR_X])
        cell_y = int(cell[INDEX_FOR_Y])
        for d in np.arange(1, self.cell_radius):  # TODO orientation
            for x in xrange(cell_x - d, cell_x + d + 1):  # go over x axis
                for y in range(cell_y - d, cell_y + d + 1):  # go over y axis
                    if self.within_boundaries(x, y):
                        angle = np.around(np.rad2deg(pu.theta(cell, [x, y])), decimals=1)  # TODO parameter
                        if angle not in shadow_angle:
                            if self.is_obstacle(x, y):
                                shadow_angle.add(angle)
                            elif self.is_unknown(x, y):
                                unknown_cells.add((x, y))

        return len(unknown_cells)

    def pose_to_grid(self, pose):
        """Pose (x,y) in header.frame_id to grid coordinates"""
        # Get transformation matrix map-occupancy grid.
        return (self.transformation_matrix_grid_map.dot([pose[0], pose[1], 0, 1]))[
               0:2] / self.resolution  # TODO check int.

    def grid_to_pose(self, grid_coordinate):
        """Pose (x,y) in grid coordinates to pose in frame_id"""
        # Get transformation matrix map-occupancy grid.
        return (self.transformation_matrix_map_grid.dot(
            np.array([grid_coordinate[0] * self.resolution,
                      grid_coordinate[1] * self.resolution, 0, 1])))[0:2]

    def get_explored_region(self):
        """ Get all the explored cells on the grid map"""

        # TODO refactor the code, right now hardcoded to quickly solve the problem of non-common areas.
        # TODO more in general use matrices.
        def nearest_multiple(number, res=0.2):
            return np.round(res * np.floor(round(number / res, 2)), 1)

        p_in_sender = PointStamped()
        p_in_sender.header = self.header

        poses = set()
        self.tf_listener.waitForTransform("robot_0/map",
                                          self.header.frame_id, rospy.Time(),
                                          rospy.Duration(4.0))
        p_in_sender.header.stamp = rospy.Time()
        for x in range(self.grid.shape[1]):
            for y in range(self.grid.shape[0]):
                if self.is_free(x, y):
                    p = self.grid_to_pose((x, y))
                    p_in_sender.point.x = p[0]
                    p_in_sender.point.y = p[1]

                    p_in_common_ref_frame = self.tf_listener.transformPoint("robot_0/map", p_in_sender).point
                    poses.add((nearest_multiple(p_in_common_ref_frame.x), nearest_multiple(p_in_common_ref_frame.y)))
        return poses


class Graph:
    def __init__(self, robot_id=-1):
        self.min_hallway_width = None
        self.height = 0
        self.width = 0
        self.pixel_desc = {}
        self.free_poses = set()
        self.free_points = {}
        self.map_resolution = 0.05
        self.plot_intersection_active = False
        self.plot_data_active = False
        self.lock = Lock()
        self.adj_list = {}
        self.edges = {}
        self.leaf_slope = {}
        self.longest = None
        self.adj_dict = {}
        self.tree_size = {}
        self.leave_dict = {}
        self.parent_dict = {}
        self.new_information = {}
        self.known_points = {}
        self.unknown_points = {}
        self.global_leaves = {}
        self.leaf_edges = {}
        self.leaf_obstacles = {}
        self.performance_data = []
        self.explored_points = set()
        self.last_intersection = None
        self.latest_map = None
        self.prev_ridge = None
        rospy.init_node("graph_node")
        self.robot_id = rospy.get_param("~robot_id")
        self.robot_type = rospy.get_param("~robot_type")

        self.robot_count = rospy.get_param("/robot_count")
        self.environment = rospy.get_param("/environment")
        self.run = rospy.get_param("/run")
        self.debug_mode = rospy.get_param("/debug_mode")
        self.method = rospy.get_param("/method")
        bs_poses = rospy.get_param("/bs_pose").split(',')
        self.bs_pose = tuple([float(v) for v in bs_poses])
        self.map_scale = rospy.get_param('/map_scale')
        self.graph_scale = rospy.get_param("/graph_scale")
        self.termination_metric = rospy.get_param("/termination_metric")
        self.frontier_threshold = rospy.get_param("/frontier_threshold")
        self.min_hallway_width = rospy.get_param("/min_hallway_width".format(self.robot_id)) * self.graph_scale
        self.comm_range = rospy.get_param("/comm_range".format(self.robot_id)) * self.graph_scale
        self.point_precision = rospy.get_param("/point_precision".format(self.robot_id))
        self.min_edge_length = rospy.get_param("/min_edge_length".format(self.robot_id)) * self.graph_scale
        self.lidar_scan_radius = rospy.get_param("/lidar_scan_radius".format(self.robot_id)) * self.graph_scale
        self.lidar_fov = rospy.get_param("/lidar_fov".format(self.robot_id))
        self.slope_bias = rospy.get_param("/slope_bias".format(self.robot_id))
        self.separation_bias = rospy.get_param("/separation_bias".format(self.robot_id)) * self.graph_scale
        self.opposite_vector_bias = rospy.get_param("/opposite_vector_bias".format(self.robot_id))
        rospy.Service('/robot_{}/rendezvous'.format(self.robot_id), RendezvousPoints,
                      self.fetch_rendezvous_points_handler)
        rospy.Service('/robot_{}/frontier_points'.format(self.robot_id), FrontierPoint, self.frontier_point_handler)
        rospy.Service('/robot_{}/explored_region'.format(self.robot_id), ExploredRegion,
                      self.fetch_explored_region_handler)
        self.get_map = rospy.ServiceProxy('/robot_{}/static_map'.format(self.robot_id), GetMap)
        if self.robot_type == 1:
            self.get_map = rospy.ServiceProxy('/robot_0/static_map'.format(self.robot_id), GetMap)
        rospy.Subscriber('/shutdown', String, self.shutdown_callback)
        self.last_frontier_points=[]
        self.listener = tf.TransformListener()
        self.last_graph_update_time = rospy.Time.now().to_sec()
        rospy.loginfo('Robot {}: Successfully created graph node'.format(self.robot_id))

    def spin(self):
        r = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            try:
                r.sleep()
            except Exception as e:
                rospy.logerr('Robot {}: Graph node interrupted!: {}'.format(self.robot_id, e))

    def generate_graph(self):
        try:
            self.latest_map = Grid(self.get_map().map)
        except rospy.ServiceException:
            pu.log_msg(self.robot_id, "Map didn't update", self.debug_mode)
            return

    def frontier_point_handler(self, request):
        count = request.count
        frontier_points = []
        self.generate_graph()
        free_points = list(self.latest_map.get_explored_region())
        if free_points:
            hull, boundary_points = graham_scan(free_points, count, False)
            for ap in boundary_points:
                pose = Pose()
                pose.position.x = ap[INDEX_FOR_X]
                pose.position.y = ap[INDEX_FOR_Y]
                frontier_points.append(pose)
        return FrontierPointResponse(frontiers=frontier_points)

    def fetch_explored_region_handler(self, data):
        self.generate_graph()
        poses = self.latest_map.get_explored_region()
        exp_points = []
        for p in poses:
            pose = Pose()
            pose.position.x = round(p[INDEX_FOR_X], 1)
            pose.position.y = round(p[INDEX_FOR_Y], 1)
            exp_points.append(pose)
        return ExploredRegionResponse(poses=exp_points, resolution=self.map_resolution)

    def fetch_rendezvous_points_handler(self, data):
        count = data.count
        self.generate_graph()
        rendezvous_points = []
        origin = self.get_robot_pose()
        free_points = []
        all_poses = self.latest_map.get_explored_region()
        for pos in all_poses:
            if pu.D(origin, pos) <= self.comm_range:
                free_points.append(pos)
        if free_points:
            hull, boundary_points = graham_scan(free_points, count, False)
            for ap in boundary_points:
                pose = Pose()
                pose.position.x = ap[INDEX_FOR_X]
                pose.position.y = ap[INDEX_FOR_Y]
                rendezvous_points.append(pose)
        if rendezvous_points:
            self.last_frontier_points=rendezvous_points
        return RendezvousPointsResponse(poses=self.last_frontier_points)

    def enough_delay(self):
        return rospy.Time.now().to_sec() - self.last_graph_update_time > 30  # updated last 20 secs

    def get_robot_pose(self):
        if self.robot_type == 1:
            rospy.logerr("Robot id: {} count: {}, pose: {}".format(self.robot_id, self.robot_count, self.bs_pose))
            return self.bs_pose
        robot_pose = None
        while not robot_pose:
            try:
                self.listener.waitForTransform("robot_{}/map".format(self.robot_id),
                                               "robot_{}/base_link".format(self.robot_id), rospy.Time(),
                                               rospy.Duration(4.0))
                (robot_loc_val, rot) = self.listener.lookupTransform("robot_{}/map".format(self.robot_id),
                                                                     "robot_{}/base_link".format(self.robot_id),
                                                                     rospy.Time(0))
                robot_pose = (math.floor(robot_loc_val[0]), math.floor(robot_loc_val[1]), robot_loc_val[2])
                sleep(1)
            except Exception as e:
                rospy.logerr(e)
                pass

        return robot_pose

    def shutdown_callback(self, msg):
        rospy.signal_shutdown('MapAnalyzer: Shutdown command received!')


if __name__ == "__main__":
    graph = Graph()
    graph.spin()
