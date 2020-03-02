#!/usr/bin/python

import time

import rospy
from std_msgs.msg import *
from std_srvs.srv import *
from nav2d_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
import math
from create_graph import Graph
from recurrent_connectivity.msg import *
from random import randint
from nav2d_navigator.msg import MoveToPosition2DActionGoal, MoveToPosition2DActionResult
from actionlib_msgs.msg import GoalStatusArray, GoalID
from recurrent_connectivity.msg import *
from std_srvs.srv import Trigger
import sys
from time import sleep
from threading import Thread, Lock
import copy
import numpy as np
import tf
import pickle
import os
from os.path import exists, getsize
from std_msgs.msg import String
import project_utils as pu

INF = 1000000000000
NEG_INF = -1000000000000
# ids to identify the robot type
BS_TYPE = 1
RR_TYPE = 2
FR_TYPE = 3

ACCEPTABLE_RANGE = 1.5

WIFI_RANGE = 20
BLUETOOTH_RANGE = 5

ROBOT_SPACE = 1

RENDEZVOUS_SEND_WINDOW = 10
MAX_ATTEMPTS = 2
WAIT_TIME = 10
BACK_TO_ORIGIN = 2
TO_RENDEZVOUS = 1
TO_FRONTIER = 0
TO_MEETING_POINT = 3
FROM_EXPLORATION = 4

ACTIVE_STATE = 1  # This state shows that the robot is collecting messages
PASSIVE_STATE = -1  # This state shows  that the robot is NOT collecting messages

# MOVE STATUSES

PENDING = 0  # The goal has yet to be processed by the action server
ACTIVE = 1  # The goal is currently being processed by the action server
PREEMPTED = 2  # The goal received a cancel request after it started executing and has since completed its execution
# (Terminal State)
SUCCEEDED = 3  # The goal was achieved successfully by the action server (Terminal State)
ABORTED = 4  # The goal was aborted during execution by the action server due to some failure (Terminal State)
REJECTED = 5  # The goal was rejected by the action server without being processed, because the goal was
# unattainable or invalid (Terminal State)
PREEMPTING = 6  # The goal received a cancel request after it started executing and has not yet completed execution
RECALLING = 7  # The goal received a cancel request before it started executing, but the action server has not yet
# confirmed that the goal is canceled and was successfully cancelled (Terminal State)
LOST = 9  # An action client can determine that a goal is LOST. This should not be sent over the wire by an action

# server

START_SCAN = '1'
STOP_SCAN = '0'
MAX_TARGET_INFO_RATIO = 0.8


class Robot:
    def __init__(self, robot_id, robot_type=0, base_stations=[], relay_robots=[], frontier_robots=[]):
        self.lock = Lock()

        # id of the robot
        self.robot_id = robot_id

        # type of robot
        self.robot_type = robot_type

        # id of base station robot (for the case of relay robot)
        self.base_stations = base_stations

        # list of relay robots (for the case of base station and frontier robots (1 RR in this case)
        self.relay_robots = relay_robots

        # list of frontier robots (for the case of relay robots)
        self.frontier_robots = frontier_robots

        # list of relevant robot ids according to the robot type
        self.candidate_robots = []

        # publishers of rendezvous points to corresponding candidate robots
        self.rendezvous_publishers = {}

        # tracking current frontiers of each Robot (used by BS)
        self.rendezvous_points = {}

        self.parent_robot_id = None

        # publish data topic of a particular relay robot (applicable to only frontier robot)
        self.map_pub = None

        # rendezvous point (created by base station for relay robots or by Relay Robot for its frontier robots)
        # this is set by the path planner
        self.rendezvous_point = None

        self.rendezvous_options = []

        self.frontier_point = None

        self.previous_frontier_point = None

        self.meeting_point = None
        self.robot_pose = None
        self.robot_state = ACTIVE_STATE  # Robot can save its initial data

        # Buffer for incoming karto_in messages before they're shared with other robots
        self.data_file_name = 'scan_data_{}.pickle'.format(self.robot_id)

        self.robot_rotating = False

        # Data structure for storing pointers to publishers
        self.publisher_map = {}

        # Flag used by BS and RR to resend rendezvous locations for the first time
        self.is_initial_rendezvous_sharing = True

        # Flag used by RR to control when to share data with other robots
        self.is_initial_rendezvous_receipt = True

        self.initial_view = None

        self.initial_data_count = 0

        self.is_exploring = False

        self.is_initial_frontier = True
        self.close_devices = []
        self.exploration_id = None
        self.frontier_points = []
        self.failed_points = []
        self.karto_messages = {}
        self.home_alert_id = None
        self.last_map_update_time = rospy.Time.now().secs
        self.last_buffered_data_time = rospy.Time.now().secs
        self.home_alert_map = {}
        self.retry_attempts = 0
        self.move_attempt = 0
        self.saved_messages = 0
        self.signal_strength = []
        self.total_collected_size = 0
        self.robot_count = rospy.get_param("~robot_count")
        self.environment = rospy.get_param("~environment")
        self.debug_mode = rospy.get_param("~debug_mode")
        self.termination_metric = rospy.get_param("~termination_metric")
        self.target_distance = rospy.get_param('~target_distance')
        self.target_angle = rospy.get_param('~target_angle')
        self.coverage = None
        self.exploration_time = rospy.Time.now().to_sec()
        run = rospy.get_param("~run")

        self.graph_processor = Graph(self.environment, self.robot_count, run,
                                     self.termination_metric, self.robot_id)

        # Creating publishers
        if robot_type == BS_TYPE:
            self.candidate_robots = self.relay_robots
            for ri in self.candidate_robots:
                pub = rospy.Publisher("/robot_{}/received_data".format(ri), BufferedData, queue_size=1000)
                self.publisher_map[ri] = pub

        elif robot_type == FR_TYPE:
            self.candidate_robots = self.relay_robots
            self.parent_robot_id = self.relay_robots[0]
            pub = rospy.Publisher("/robot_{}/received_data".format(self.parent_robot_id), BufferedData,
                                  queue_size=1000)
            self.publisher_map[self.parent_robot_id] = pub

        elif robot_type == RR_TYPE:
            self.candidate_robots = self.frontier_robots + self.base_stations + self.relay_robots
            self.parent_robot_id = self.base_stations[0]
            for rid in self.candidate_robots:
                pub = rospy.Publisher("/robot_{}/received_data".format(rid), BufferedData, queue_size=1000)
                alert_pub = rospy.Publisher("/robot_{}/home_alert".format(rid), HomeAlert, queue_size=1)
                self.home_alert_map[rid] = alert_pub
                self.publisher_map[rid] = pub

        # subscribing to the rendezvous location topic in order to receiver rendezvous location updates -- for RRs & FRs
        if self.robot_type != BS_TYPE:
            rospy.Subscriber('/roscbt/robot_{}/rendezvous_points'.format(self.robot_id), RendezvousPoints,
                             self.callback_rendezvous_points)

        # subscribe to the robot's scan topic
        rospy.Subscriber('/karto_out', LocalizedScan, self.robots_karto_out_callback, queue_size=1000)
        self.chose_point_pub = rospy.Publisher("/chosen_point", ChosenPoint, queue_size=1000)
        self.karto_pub = rospy.Publisher("/robot_{}/karto_in".format(self.robot_id), LocalizedScan, queue_size=1000)
        rospy.Subscriber("/chosen_point", ChosenPoint, self.chosen_point_callback)
        rospy.Subscriber("/robot_{}/home_alert".format(self.robot_id), HomeAlert, self.home_alert_callback)
        self.received_choices = {}
        self.arrival_count = 0

        self.auction_point_pub = rospy.Publisher("/auction_point", ChosenPoint, queue_size=1000)
        rospy.Subscriber("/robot_{}/navigator/plan".format(self.robot_id), GridCells, self.navigation_plan_callback)
        self.navigation_plan = None
        self.waiting_for_plan = False
        rospy.Subscriber("/robot_{}/start_exploration".format(self.parent_robot_id), String,
                         self.start_exploration_callback)

        self.start_exploration_pub = rospy.Publisher("/robot_{}/start_exploration".format(self.robot_id), String,
                                                     queue_size=10)
        self.start_now = None

        # subscribe to incoming messages. Data is not necessary since its handled in mapper. Just used to as a trigger
        rospy.Subscriber('/roscbt/robot_{}/received_data'.format(self.robot_id), BufferedData,
                         self.buffered_data_callback, queue_size=100)

        # subscribing to the robots' pose topics (relay robots if required)
        for i in self.candidate_robots:
            # publisher of rendezvous locations. No robot sends rendezvous location to its parent
            if i != self.parent_robot_id:
                pub = rospy.Publisher("/robot_{}/rendezvous_points".format(i), RendezvousPoints, queue_size=10)
                self.rendezvous_publishers[i] = pub

        # subscribe to the robot map data for determining frontiers
        rospy.Subscriber("/robot_{}/map".format(self.robot_id), OccupancyGrid, self.map_callback)
        rospy.Subscriber("/roscbt/robot_{}/signal_strength".format(self.robot_id), SignalStrength,
                         self.signal_strength_callback)
        rospy.Subscriber("/robot_{}/pose".format(self.robot_id), Pose, callback=self.pose_callback)
        self.pose_publisher = rospy.Publisher("/robot_{}/cmd_vel".format(self.robot_id), Twist, queue_size=1)
        # Navigation and Exploration: This block implements all topics and services required for exploration

        # counts of goals visited
        self.goal_count = 0

        # number of shared rendezvous points
        self.shared_rendezvous_count = 0

        self.shared_data = {}

        self.returned_fr_count = 0

        self.sent_fr_count = 0

        # only used by RR and FR to avoid duplicate move result callbacks
        self.move_result_data = None
        self.move_status_data = None

        # only used by FR to keep track of exploration time
        self.exploration_start_time = None  # record time the robot started to explore
        self.total_exploration_time = None  # maximum amount the robot should take to explore

        self.navigation_start_time = None

        self.moveTo_pub = None
        self.start_exploration = None
        self.stop_exploration = None
        self.move_to_stop = None
        self.cancel_explore_pub = None

        # time reference algorithm parameters
        self.base_map_points = {}
        self.latest_map = None
        self.info_base = 0
        self.info_base_pub_map = None
        self.new_info = 0
        self.targe_info_ratio = 0
        self.heading_back = 0
        self.run = rospy.get_param("~run")
        self.termination_metric = rospy.get_param("~termination_metric")
        self.max_exploration_time = rospy.get_param("~max_exploration_time")
        self.max_coverage = rospy.get_param("~max_coverage")
        self.max_common_coverage = rospy.get_param("~max_common_coverage")
        if self.robot_type != BS_TYPE:
            # subscribe to status of the navigation
            rospy.Subscriber("/robot_{}/MoveTo/status".format(self.robot_id), GoalStatusArray,
                             self.move_status_callback)

            # subscribe to the result of the navigation
            rospy.Subscriber("/robot_{}/MoveTo/result".format(self.robot_id), MoveToPosition2DActionResult,
                             self.move_result_callback)

            # moveTo publisher used to send a robot to a particular location
            self.moveTo_pub = rospy.Publisher("/robot_{}/MoveTo/goal".format(self.robot_id), MoveToPosition2DActionGoal,
                                              queue_size=10)
            # start exploration service used to trigger a robot to start exploration
            self.start_exploration = rospy.ServiceProxy('/robot_{}/StartExploration'.format(self.robot_id), Trigger)

            # stop exploration service used to trigger a robot to stop exploration
            rospy.Subscriber('/robot_{}/Explore/status'.format(self.robot_id), GoalStatusArray,
                             self.exploration_callback)

            self.fetch_frontier_points = rospy.ServiceProxy('/robot_{}/frontier_points'.format(self.robot_id),FrontierPoint)
            self.fetch_rendezvous_points = rospy.ServiceProxy('/robot_{}/rendezvous_points'.format(self.robot_id),RendezvousPoints)

            self.cancel_explore_pub = rospy.Publisher("/robot_{}/Explore/cancel".format(self.robot_id), GoalID,queue_size=10)

            # stop service used to trigger a robot to stop moving
            self.move_to_stop = rospy.ServiceProxy('/robot_{}/Stop'.format(self.robot_id), Trigger)

        # == Navigation and Exploration ends here ==

        # robot identification sensor
        self.robot_range_pub = rospy.Publisher("/robot_{}/robot_ranges".format(self.robot_id), RobotRange, queue_size=5)
        self.robot_poses = {}
        self.trans_matrices = {}
        self.inverse_trans_matrices = {}
        self.latest_points = {}
        self.base_points = {}
        all_robots = self.candidate_robots + [self.robot_id]
        for i in all_robots:
            s = "def a_" + str(i) + "(self, data): self.robot_poses[" + str(i) + "] = (data.pose.pose.position.x," \
                                                                                 "data.pose.pose.position.y," \
                                                                                 "(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)) "
            exec(s)
            exec("setattr(Robot, 'callback_pos_teammate" + str(i) + "', a_" + str(i) + ")")
            exec("rospy.Subscriber('/robot_" + str(
                i) + "/base_pose_ground_truth', Odometry, self.callback_pos_teammate" + str(i) + ", queue_size = 100)")

        self.shutdown_pub = rospy.Publisher("/shutdown".format(self.robot_id), String, queue_size=10)
        rospy.Subscriber('/shutdown', String, self.shutdown_callback)
        rospy.Subscriber('/coverage'.format(self.robot_id), Coverage, self.coverage_callback)
        rospy.on_shutdown(self.save_all_data)
        self.exploration_started = False
        self.is_shutdown_caller = False
        rospy.on_shutdown(self.save_all_data)
        # ======= pose transformations====================
        self.listener = tf.TransformListener()

        rospy.loginfo("Robot {} Initialized successfully!!".format(self.robot_id))

    def spin(self):
        r = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            if self.exploration_started:
                time_to_shutdown = self.evaluate_exploration()
                if time_to_shutdown:
                    self.cancel_exploration()
                    rospy.signal_shutdown('Exploration complete!')
                else:
                    self.publish_robot_ranges()
                    if self.robot_type == RR_TYPE and self.is_exploring:
                        self.check_if_time_to_getback_to_rendezvous()
            r.sleep()

    def coverage_callback(self, data):
        self.coverage = data

    def evaluate_exploration(self):
        its_time = False
        if self.coverage:
            if self.termination_metric == pu.MAXIMUM_EXPLORATION_TIME:
                time = rospy.Time.now().to_sec() - self.exploration_time
                its_time = time >= self.max_exploration_time * 60
            elif self.termination_metric == pu.TOTAL_COVERAGE:
                its_time = self.coverage.coverage >= self.max_coverage
            elif self.termination_metric == pu.FULL_COVERAGE:
                its_time = self.coverage.coverage >= self.coverage.expected_coverage
            elif self.termination_metric == pu.COMMON_COVERAGE:
                its_time = self.coverage.common_coverage >= self.max_common_coverage

        return its_time

    def wait_for_map_update(self):
        r = rospy.Rate(1)
        while not self.is_time_to_moveon():
            r.sleep()

    def signal_strength_callback(self, data):
        signals = data.signals
        robots = []
        devices = []
        for rs in signals:
            robots.append([rs.robot_id, rs.rssi])
            devices.append(str(rs.robot_id))
        self.signal_strength = robots
        self.close_devices = devices

    ''' This publishes the newly computed rendezvous points for 10 seconds in an interval of 1 second'''

    def publish_rendezvous_points(self, rendezvous_poses, receivers, direction=1, total_exploration_time=0):
        rv_points = RendezvousPoints()
        rv_points.poses =rendezvous_poses
        rv_points.msg_header.header.frame_id = '{}'.format(self.robot_id)
        rv_points.msg_header.sender_id = str(self.robot_id)
        rv_points.msg_header.topic = 'rendezvous_points'
        rv_points.exploration_time = total_exploration_time
        rv_points.direction = direction
        for id in receivers:
            rv_points.msg_header.receiver_id = str(id)
            self.rendezvous_publishers[id].publish(rv_points)

    def robots_karto_out_callback(self, data):
        if data.robot_id - 1 == self.robot_id:
            for rid in self.candidate_robots:
                self.add_to_file(rid, [data])
            if self.robot_type == RR_TYPE and self.is_initial_rendezvous_sharing:
                self.is_initial_rendezvous_sharing = False
                self.push_messages_to_receiver([self.parent_robot_id])

    def callback_rendezvous_points(self, data):
        if data.header.frame_id == str(self.parent_robot_id):
            rospy.logerr("Robot {} received points from {}".format(self.robot_id, self.parent_robot_id))
            x = data.x
            y = data.y
            direction = data.direction
            received_points = [(x[i], y[i]) for i in range(len(x))]
            self.wait_for_map_update()
            self.move_attempt = 0
            robot_pose = self.get_robot_pose()
            valid_points = self.graph_processor.get_optimal_points(robot_pose, received_points)
            optimal_points = self.get_available_points(valid_points)
            if optimal_points:
                new_point = optimal_points[0]
            else:
                new_point = received_points[0]

            if not self.frontier_point:
                self.frontier_point = robot_pose

            if direction == TO_RENDEZVOUS:
                self.rendezvous_point = copy.deepcopy(new_point)
                self.rendezvous_points = optimal_points
                self.move_robot_to_goal(self.rendezvous_point, TO_RENDEZVOUS)
                rospy.logerr("Robot {} going to RV point: {}".format(self.robot_id, self.rendezvous_point))
            elif direction == TO_FRONTIER:
                self.frontier_points = optimal_points
                self.previous_frontier_point = copy.deepcopy(self.frontier_point)
                self.frontier_point = new_point
                self.total_exploration_time = data.exploration_time
                self.move_robot_to_goal(self.frontier_point, TO_FRONTIER)
                rospy.logerr("Robot {} going to frontier: {}".format(self.robot_id, self.frontier_point))

    def map_callback(self, data):
        # self.add_map_to_file(self.robot_id, [data])
        self.last_map_update_time = rospy.Time.now().secs
        if self.robot_type == BS_TYPE:
            self.get_known_regions(data, self.base_map_points)
        else:
            self.latest_map = data
            if len(self.base_map_points):
                self.update_info()
        self.graph_processor.update_graph(data)

    def rotate_robot(self):
        deg_360 = math.radians(360)
        if self.robot_pose:
            while self.robot_pose[2] < deg_360:
                self.move_robot((0, 1))

    def move_robot(self, vel):
        vel_msg = Twist()
        vel_msg.linear.x = vel[0]
        vel_msg.angular.z = vel[1]
        self.pose_publisher.publish(vel_msg)

    '''This is executed by the BS and FR to determine whether its time to move back to a rendezvous location and 
    share information '''

    def check_if_time_to_getback_to_rendezvous(self):
        if self.robot_type == RR_TYPE:
            if self.robot_id == 1:
                rospy.logerr("Robot {} TargetInfoRatio: {}".format(self.robot_id, self.targe_info_ratio))
            if self.is_exploring:
                self.compute_target_ratio()
                if self.targe_info_ratio > MAX_TARGET_INFO_RATIO:
                    # rospy.logerr("Robot {}: Reached target ratio: {}".format(self.robot_id, self.targe_info_ratio))
                    self.move_back_to_base_station()
                else:
                    # check if there is any robot around and share data
                    if self.home_alert_id in self.close_devices:
                        rospy.logerr("Robot {}: Hotspots: {}".format(self.robot_id, self.close_devices))
                        self.push_messages_to_receiver(self.close_devices)
                    self.home_alert_id = None

    ''' Helper method to publish a message to any receiver karto_in topic'''

    def push_messages_to_receiver(self, receiver_ids, is_alert=0):
        for rid in receiver_ids:
            message_data = self.load_data_for_id(rid)
            if message_data:
                buffered_data = BufferedData()
                buffered_data.msg_header.header.stamp.secs = rospy.Time.now().secs
                buffered_data.msg_header.header.frame_id = '{}'.format(self.robot_id)
                buffered_data.msg_header.sender_id = str(self.robot_id)
                buffered_data.msg_header.receiver_id = str(rid)
                buffered_data.msg_header.topic = 'received_data'
                buffered_data.secs = []
                buffered_data.data = message_data
                x = []
                y = []
                base_ps = list(self.base_map_points)
                if len(base_ps):
                    x, y = zip(*base_ps)
                buffered_data.base_x = x
                buffered_data.base_y = y
                buffered_data.heading_back = self.heading_back
                buffered_data.alert_flag = is_alert
                self.publisher_map[rid].publish(buffered_data)
                self.delete_data_for_id(rid)  ## TOFIX?

        self.base_map_points.update(self.latest_points)

    ''' Helper method to publish a message to MoveTo/goal topic'''

    def move_robot_to_goal(self, goal, direction=1):
        id_val = "robot_{}_{}_{}".format(self.robot_id, self.goal_count, direction)
        move = MoveToPosition2DActionGoal()
        move.header.frame_id = '/robot_{}/map'.format(self.robot_id)
        goal_id = GoalID()
        goal_id.id = id_val
        move.goal_id = goal_id
        move.goal.target_pose.x = goal[0]
        move.goal.target_pose.y = goal[1]
        move.goal.target_distance = self.target_distance
        move.goal.target_angle = self.target_angle
        self.moveTo_pub.publish(move)
        self.goal_count += 1
        if self.robot_type == RR_TYPE and direction == BACK_TO_ORIGIN:
            self.navigation_start_time = rospy.Time.now()

        chosen_point = ChosenPoint()
        chosen_point.header.frame_id = '{}'.format(self.robot_id)
        chosen_point.x = goal[0]
        chosen_point.y = goal[1]
        chosen_point.direction = direction
        self.chose_point_pub.publish(chosen_point)

        self.robot_state = PASSIVE_STATE

    def move_back_to_base_station(self):
        self.cancel_exploration()
        self.heading_back = 1
        self.is_exploring = False
        robot_pose = self.get_robot_pose()
        # if self.parent_robot_id in self.close_devices:
        #     self.rendezvous_point = robot_pose
        #     self.push_messages_to_receiver([self.parent_robot_id])
        # else:
        #     rospy.logerr("Robot {} heading back to RV point: {}".format(self.robot_id, self.rendezvous_point))
        points = self.graph_processor.get_optimal_points(robot_pose, self.rendezvous_points)
        available_points = self.get_available_points(points)
        if available_points:
            self.rendezvous_point = available_points[0]
        self.move_robot_to_goal(self.rendezvous_point, FROM_EXPLORATION)

    ''' This is a call back that receives the status of the robot during navigation'''

    def move_status_callback(self, data):
        id_4 = "robot_{}_{}_{}".format(self.robot_id, self.goal_count - 1, FROM_EXPLORATION)
        id_1 = "robot_{}_{}_{}".format(self.robot_id, self.goal_count - 1, TO_RENDEZVOUS)
        id_0 = "robot_{}_{}_{}".format(self.robot_id, self.goal_count - 1, TO_FRONTIER)
        if data.status_list:
            goal_status = data.status_list[0]
            if goal_status.goal_id.id:
                if goal_status.goal_id.id == id_4:
                    if goal_status.status == ACTIVE or goal_status.status == ABORTED:
                        if self.parent_robot_id in self.close_devices:
                            # rospy.logerr("Heading  back from exploration.. stopped: there's connectivity")
                            self.move_to_stop()
                            pose = self.get_robot_pose()
                            self.rendezvous_point = pose
                            self.push_messages_to_receiver([self.parent_robot_id])
                        else:
                            for rid in self.close_devices:
                                alert = HomeAlert()
                                alert.robot_id = self.robot_id
                                alert.home_alert = 1
                                self.home_alert_map[rid].publish(alert)
                elif goal_status.goal_id.id == id_0:
                    if goal_status.status == ACTIVE:
                        # rospy.logerr("Robot type, {} Heading to Frontier point...".format(self.robot_type))
                        if goal_status.status == ABORTED:
                            result = self.start_exploration()
                            if result:  # start exploration anyway
                                self.exploration_start_time = rospy.Time.now()
                                self.is_exploring = True
                                self.robot_state = ACTIVE_STATE
                elif goal_status.goal_id.id == id_4:
                    if goal_status.status == REJECTED:
                        rospy.logerr("Robot type, {} Heading back from exploration...".format(self.robot_type))

    '''This callback receives the final result of the most recent navigation to the goal'''

    def move_result_callback(self, data):
        id_4 = "robot_{}_{}_{}".format(self.robot_id, self.goal_count - 1, FROM_EXPLORATION)
        id_1 = "robot_{}_{}_{}".format(self.robot_id, self.goal_count - 1, TO_RENDEZVOUS)
        id_0 = "robot_{}_{}_{}".format(self.robot_id, self.goal_count - 1, TO_FRONTIER)

        if data.status:
            if data.status.status == ABORTED:
                if data.status.goal_id.id == id_1:
                    if self.move_attempt < MAX_ATTEMPTS:
                        # rospy.logerr("Robot {}: Navigation failed. Trying again..".format(self.robot_id))
                        self.rotate_robot()
                        self.move_robot_to_goal(self.rendezvous_point, TO_RENDEZVOUS)
                        self.move_attempt += 1
                    else:
                        # rospy.logerr("Robot {}: To rendezvous Navigation failed!".format(self.robot_id))
                        self.failed_points.append(self.rendezvous_point)
                        rv_ps = [p for p in self.rendezvous_points if p not in self.failed_points]
                        if rv_ps:
                            self.rendezvous_point = rv_ps[-1]
                            self.move_robot_to_goal(self.rendezvous_point, TO_RENDEZVOUS)
                        else:
                            self.initiate_exploration()
                elif data.status.goal_id.id == id_0:
                    result = self.start_exploration()
                    if result:
                        self.exploration_start_time = rospy.Time.now()
                        self.is_exploring = True
                        self.robot_state = ACTIVE_STATE
                        self.heading_back = 0

            elif data.status.status == SUCCEEDED:
                self.retry_attempts = 0
                if data.status.goal_id.id == id_1:  # TO RV (RR or FR)
                    if self.robot_type == RR_TYPE:
                        rospy.logerr("Robot {}  Arrived at RV point: {}".format(self.robot_id, self.rendezvous_point))
                        self.heading_back = 0
                        # self.push_messages_to_receiver([self.parent_robot_id])
                        self.initiate_exploration()
                    elif self.robot_type == FR_TYPE:
                        rospy.logerr("Robot {}  Arrived at RV point: {}".format(self.robot_id, self.rendezvous_point))
                        self.push_messages_to_receiver([self.parent_robot_id], is_alert=1)

                elif data.status.goal_id.id == id_4:  # to meeting point for FRs
                    rospy.logerr("Robot {} back from exploration..".format(self.robot_id))
                    self.push_messages_to_receiver([self.parent_robot_id])

                elif data.status.goal_id.id == id_0:
                    if self.robot_type == RR_TYPE:
                        rospy.logerr("Robot {} arrived at Frontier: {}".format(self.robot_id, self.frontier_point))
                        self.initiate_exploration()

    def initiate_exploration(self):
        result = self.start_exploration()
        if result:
            self.exploration_start_time = rospy.Time.now()
            self.is_exploring = True
            self.heading_back = 0
            self.robot_state = ACTIVE_STATE

    def process_data(self, sender_id, buff_data):
        data_vals = buff_data.data
        self.last_map_update_time = rospy.Time.now().secs
        counter = 0
        r = rospy.Rate(1)
        for scan in data_vals:
            self.karto_pub.publish(scan)
            counter += 1
            r.sleep()

        if self.robot_type == BS_TYPE:
            for rid in self.relay_robots:
                if rid != sender_id:
                    self.add_to_file(rid, data_vals)

        elif self.robot_type == RR_TYPE:
            if sender_id in self.frontier_robots:
                self.add_to_file(self.parent_robot_id, data_vals)
            elif sender_id == self.parent_robot_id:  # and self.is_initial_rendezvous_receipt:
                for rid in self.frontier_robots:
                    self.add_to_file(rid, data_vals)

    '''This callback receives the incoming karto_in data'''

    def buffered_data_callback(self, buff_data):
        sender_id = buff_data.msg_header.header.frame_id
        if sender_id in self.candidate_robots:
            rospy.logerr("Robot {} received data from {}".format(self.robot_id, sender_id))
            self.process_data(sender_id, buff_data)
        # self.wait_for_map_update()
        if self.robot_type == BS_TYPE:
            robot_pose = self.get_robot_pose()
            self.push_messages_to_receiver([sender_id])
            direction = TO_RENDEZVOUS
            if self.is_initial_rendezvous_sharing:
                self.initial_data_count += 1
                if self.initial_data_count == len(self.relay_robots):
                    self.wait_for_map_update()
                    rendezvous_points = self.graph_processor.get_rendezvous_points(robot_pose, WIFI_RANGE)
                    if rendezvous_points:
                        self.publish_rendezvous_points(rendezvous_points, self.relay_robots, direction=direction)
                        self.is_initial_rendezvous_sharing = False
                    else:
                        rospy.logerr("Robot {}: No rendzvous points to send...".format(self.robot_id))
            # else:
            #     direction = TO_FRONTIER
            #     self.wait_for_map_update()
            #     frontier_points = self.graph_processor.get_frontiers()
            #     if frontier_points:
            #         self.frontier_points = frontier_points
            #         self.publish_rendezvous_points(frontier_points, [sender_id], direction=direction)
            #     else:
            #         rospy.logerr("Robot {}: No frontier points to send...".format(self.robot_id))

        elif self.robot_type == RR_TYPE:
            map_points = self.get_map_points(buff_data)
            if sender_id == self.parent_robot_id:
                self.base_map_points = map_points
            elif sender_id in self.relay_robots:
                heading_back = buff_data.heading_back
                if heading_back:
                    # then you've shared your data and you're all set. continue with exploration
                    if self.heading_back:
                        self.move_to_stop()
                        self.initiate_exploration()
                    self.base_map_points = copy.deepcopy(self.latest_points)  # assume BS is updated
                else:
                    # evaluate whether or not to head back
                    known_region = len(map_points)
                    if known_region > self.info_base:  # update info base if the incoming is more updated
                        self.base_map_points = map_points
                    self.update_info()
                    if self.targe_info_ratio > MAX_TARGET_INFO_RATIO:
                        self.move_back_to_base_station()  # head back to base station
                    self.push_messages_to_receiver([sender_id])

    def get_map_points(self, buff_data):
        map_points = {}
        X = list(buff_data.base_x)
        Y = list(buff_data.base_y)
        h = len(X)
        for i in range(h):
            map_points[(X[i], Y[i])] = None
        return map_points

    def home_alert_callback(self, data):
        rospy.logerr('Robot {}: Home Alert received from {}'.format(self.robot_id, data.robot_id))
        self.home_alert_id = str(data.robot_id)

    def update_info(self):
        robot_known_region = self.get_known_regions(self.latest_map, self.latest_points)
        base_known_region = set(list(self.base_map_points))
        self.info_base = float(len(base_known_region))
        new_region = robot_known_region.intersection(base_known_region)
        self.new_info = len(robot_known_region) - len(new_region)
        self.compute_target_ratio()

    def chosen_point_callback(self, data):
        self.received_choices[(data.x, data.y)] = data

    def get_available_points(self, points):
        available_points = []
        for p in points:
            if p not in self.received_choices:
                if len(self.received_choices):
                    for point in self.received_choices:
                        distance = math.sqrt((p[0] - point[0]) ** 2 + (p[1] - point[1]) ** 2)
                        if distance > ROBOT_SPACE:
                            available_points.append(p)
                else:
                    available_points.append(p)
        return available_points

    def navigation_plan_callback(self, data):
        if self.waiting_for_plan:
            self.navigation_plan = data.cells

    def start_exploration_callback(self, data):
        self.start_now = data

    def get_total_navigation_time(self, origin, plan):
        total_time = 60  # 1 min by default
        if plan:
            ordered_points = self.graph_processor.get_distances(origin, plan)
            if ordered_points:
                farthest_point = ordered_points[-1]
                distance = math.sqrt((origin[0] - farthest_point[0]) ** 2 + (origin[1] - farthest_point[1]) ** 2)
                total_time = 2 * distance / 1.0  # distance/speed
        return total_time

    def is_time_to_moveon(self):
        diff = rospy.Time.now().secs - self.last_map_update_time
        return diff > 10

    def theta(self, p, q):
        dx = q[0] - p[0]
        dy = q[1] - p[1]
        if dx == 0:
            dx = 0.000001
        return math.atan2(dy, dx)

    def D(self, p, q):
        dx = q[0] - p[0]
        dy = q[1] - p[1]
        return math.sqrt(dx ** 2 + dy ** 2)

    def get_robot_pose(self):
        robot_pose = None
        while not robot_pose:
            try:
                self.listener.waitForTransform("robot_{}/map".format(self.robot_id),
                                               "robot_{}/base_link".format(self.robot_id), rospy.Time(),
                                               rospy.Duration(4.0))
                (robot_loc_val, rot) = self.listener.lookupTransform("robot_{}/map".format(self.robot_id),
                                                                     "robot_{}/base_link".format(self.robot_id),
                                                                     rospy.Time(0))

                robot_pose = (math.floor(robot_loc_val[0]), math.floor(robot_loc_val[1]))
                sleep(1)
            except:
                rospy.logerr("Robot {}: Can't fetch robot pose from tf".format(self.robot_id))
                pass

        return robot_pose

    def get_known_regions(self, map, original_points):
        map_info = map.info
        map_width = map_info.width
        map_height = map_info.height
        grid_values = np.array(map.data).reshape((map_height, map_width)).astype(np.float32)
        for row in range(map_height):
            for col in range(map_width):
                index = [0] * 2
                index[1] = row
                index[0] = col
                data_val = grid_values[row, col]
                if data_val == 100 or data_val == 0:
                    original_points[tuple(index)] = None
        base_known_points = set(list(original_points.keys()))
        return base_known_points

    def compute_target_ratio(self):
        if self.info_base or self.new_info:
            self.targe_info_ratio = 1 - (self.info_base / float(self.new_info + self.info_base))
            # rospy.logerr("Robot {}: Target ratio: {}".format(self.robot_id, self.targe_info_ratio))

    def pose_callback(self, msg):
        pose = (msg.x, msg.y, msg.theta)
        self.robot_pose = pose

    def exploration_callback(self, data):
        if data.status_list:
            goal_status = data.status_list[0]
            gid = goal_status.goal_id.id
            if gid and self.exploration_id != gid:
                self.exploration_id = gid

    def cancel_exploration(self):
        if self.exploration_id:
            rospy.logerr("Robot {} Cancelling exploration...".format(self.robot_id))
            goal_id = GoalID()
            goal_id.id = self.exploration_id
            self.cancel_explore_pub.publish(goal_id)
        else:
            rospy.logerr("Exploration ID not set...")

    def publish_robot_ranges(self):
        if len(self.robot_poses) == len(self.candidate_robots) + 1:
            self.generate_transformation_matrices()
            if len(self.trans_matrices) == len(self.candidate_robots) + 1:
                pose = self.robot_poses[self.robot_id]
                yaw = self.get_elevation(pose[2])
                robotV = np.asarray([pose[0], pose[1], yaw, 1])
                distances = []
                angles = []
                for rid in self.candidate_robots:
                    robot_c = self.get_robot_coordinate(rid, robotV)
                    distance = self.D(pose, robot_c)
                    angle = self.theta(pose, robot_c)
                    distances.append(distance)
                    angles.append(angle)
                robot_range = RobotRange()
                robot_range.distances = distances
                robot_range.angles = angles
                robot_range.robot_id = self.robot_id
                robot_range.header.stamp = rospy.Time.now()
                self.robot_range_pub.publish(robot_range)

    def get_elevation(self, quaternion):
        euler = tf.transformations.euler_from_quaternion(quaternion)
        # roll = euler[0]
        # pitch = euler[1]
        yaw = euler[2]
        return yaw

    def parse_frontier_response(self, data):
        frontier_points = []
        received_poses = data.poses
        if received_poses:
            for p in received_poses:
                yaw = self.get_elevation((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
                frontier_points.append((p.position.x, p.position.y, yaw))
        return frontier_points

    def request_and_share_frontiers(self):
        frontier_point_response = self.fetch_frontier_points(FrontierPointRequest(count=len(self.candidate_robots) + 1))
        frontier_points = self.parse_frontier_response(frontier_point_response)
        if frontier_points:
            self.frontier_points = frontier_points




    def generate_transformation_matrices(self):
        map_origin = (0, 0, 0, 1)
        for rid in self.candidate_robots + [self.robot_id]:
            p = self.robot_poses[int(rid)]
            theta = self.theta(map_origin, p)
            M = [[np.cos(theta), -1 * np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]]
            T = copy.deepcopy(M)
            T.append([0, 0, 0, 1])
            yaw = self.get_elevation(p[2])
            T[0].append(p[0])
            T[1].append(p[1])
            T[2].append(yaw)
            tT = np.asarray(T)
            self.trans_matrices[rid] = tT
            tT_inv = np.linalg.inv(tT)
            self.inverse_trans_matrices[int(rid)] = tT_inv

    def get_robot_coordinate(self, rid, V):
        other_T = self.trans_matrices[rid]  # oTr2
        robot_T = self.inverse_trans_matrices[self.robot_id]  #
        cTr = other_T.dot(robot_T)
        # cTr_inv = np.linalg.inv(cTr)
        other_robot_pose = cTr.dot(V)
        return other_robot_pose.tolist()

    def add_map_to_file(self, rid, data):
        self.lock.acquire()
        if int(rid) == 1:
            filename = "map_messages1.pickle"
            saved_data = {}
            if not exists(filename):
                os.remove(filename)
                f = open(filename, "wb+")
                f.close()
            with open(filename, 'wb') as fp:
                pickle.dump(data[0], fp, protocol=pickle.HIGHEST_PROTOCOL)
            rospy.logerr("Robot {} Saved map message..".format(self.robot_id))
        sleep(2)
        self.lock.release()
        return True

    def add_to_file(self, rid, data):
        self.lock.acquire()
        if rid in self.karto_messages:
            self.karto_messages[rid] += data
        else:
            self.karto_messages[rid] = data
        if self.is_exploring:
            self.total_collected_size += sys.getsizeof(data[0])
        self.lock.release()
        return True

    def load_data_for_id(self, rid):
        message_data = []
        if rid in self.karto_messages:
            message_data = self.karto_messages[rid]
        return message_data

    def load_data(self):
        return self.karto_messages

    def delete_data_for_id(self, rid):
        self.lock.acquire()
        if rid in self.karto_messages:
            del self.karto_messages[rid]

        sleep(2)
        self.lock.release()
        return True

    def get_angle(self, a, b):
        new_b = b
        if b == 0:
            new_b = INF
        rel_angle = math.atan(a / new_b)
        if a > 0 and b == 0:  # end of 1st Quad
            rel_angle = math.radians(90)
        elif a < 0 and b == 0:  # start of 4th Quad
            rel_angle = math.radians(270)
        if a >= 0 > b:  # 2nd Quad
            rel_angle = math.radians(180) - rel_angle
        elif a <= 0 and b < 0:  # 3rd Quad
            rel_angle = math.radians(180) + rel_angle
        elif a <= 0 < b:  # 4th Quad
            rel_angle = math.radians(270) + rel_angle
        return rel_angle

    def save_all_data(self):
        self.graph_processor.save_all_data()
        msg = String()
        msg.data = '{}'.format(self.robot_id)
        if not self.is_shutdown_caller:
            self.is_shutdown_caller = True
            self.shutdown_pub.publish(msg)

    def shutdown_callback(self, data):
        if not self.is_shutdown_caller:
            self.is_shutdown_caller = True
            rospy.signal_shutdown('Robot {}: Received Shutdown Exploration complete!'.format(self.robot_id))


if __name__ == "__main__":
    # initializing a node
    rospy.init_node("robot_node", anonymous=True)
    robot_id = int(rospy.get_param("~robot_id", 0))
    robot_type = int(rospy.get_param("~robot_type", 0))
    base_stations = str(rospy.get_param("~base_stations", ''))
    relay_robots = str(rospy.get_param("~relay_robots", ''))
    frontier_robots = str(rospy.get_param("~frontier_robots", ''))

    relay_robots = relay_robots.split(',')
    frontier_robots = frontier_robots.split(',')
    base_stations = base_stations.split(',')

    rospy.loginfo("ROBOT_ID: {},RELAY ROBOTS: {}, FRONTIER ROBOTS: {}, BASE STATION: {}".format(robot_id, relay_robots,
                                                                                                frontier_robots,
                                                                                                base_stations))
    if relay_robots[0] == '':
        relay_robots = []
    if frontier_robots[0] == '':
        frontier_robots = []
    if base_stations[0] == '':
        base_stations = []
    robot = Robot(robot_id, robot_type=robot_type, base_stations=base_stations, relay_robots=relay_robots,
                  frontier_robots=frontier_robots)
    robot.spin()
