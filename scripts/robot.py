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
from gvgexploration.msg import *
from nav2d_navigator.msg import MoveToPosition2DActionGoal, MoveToPosition2DActionResult, ExploreActionGoal, \
    ExploreActionResult
from actionlib_msgs.msg import GoalStatusArray, GoalID
from gvgexploration.srv import *
from std_srvs.srv import Trigger
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


class Robot:
    def __init__(self, robot_id, robot_type=0, base_stations=[], relay_robots=[], frontier_robots=[]):

        self.lock = Lock()
        self.robot_id = robot_id
        self.robot_type = robot_type
        self.base_stations = base_stations
        self.relay_robots = relay_robots
        self.frontier_robots = frontier_robots
        self.rendezvous_publishers = {}
        self.rendezvous_points = []
        self.all_feedbacks = {}
        self.parent_robot_id = None
        self.map_pub = None
        self.rendezvous_point = None
        self.rendezvous_options = []
        self.frontier_point = None
        self.previous_frontier_point = None
        self.meeting_point = None
        self.robot_pose = None
        self.robot_state = ACTIVE_STATE  # Robot can save its initial data
        self.robot_rotating = False
        self.publisher_map = {}
        self.is_initial_rendezvous_sharing = True
        self.is_initial_rendezvous_receipt = True
        self.initial_view = None
        self.initial_data_count = 0
        self.is_exploring = False
        self.robot_stopped = False
        self.is_initial_frontier = True
        self.exploration_id = None
        self.frontier_points = []
        self.failed_points = []
        self.karto_messages = {}
        self.frontier_data = []
        self.home_alert_id = None
        self.last_map_update_time = rospy.Time.now().secs
        self.last_buffered_data_time = rospy.Time.now().secs
        self.home_alert_map = {}
        self.retry_attempts = 0
        self.move_attempt = 0
        self.saved_messages = 0
        self.signal_strength = []
        self.total_collected_size = 0
        self.navigation_plan = None
        self.waiting_for_plan = False
        self.start_now = None
        self.received_choices = {}
        self.arrival_count = 0
        self.goal_count = 0
        self.shared_rendezvous_count = 0
        self.shared_data = {}
        self.returned_fr_count = 0
        self.sent_fr_count = 0
        self.move_result_data = None
        self.move_status_data = None
        self.exploration_start_time = None  # record time the robot started to explore
        self.total_exploration_time = None  # maximum amount the robot should take to explore
        self.navigation_start_time = None
        self.moveTo_pub = None
        self.stop_exploration = None
        self.move_to_stop = None
        self.cancel_explore_pub = None
        self.info_base = 0
        self.info_base_pub_map = None
        self.new_info = 0
        self.target_info_ratio = 0
        self.heading_back = 0
        self.robot_poses = {}
        self.trans_matrices = {}
        self.inverse_trans_matrices = {}
        self.robot_points = set()
        self.base_points = set()
        self.shared_data_srv_map = {}
        self.shared_point_srv_map = {}
        self.allocation_srv_map = {}
        self.exploration_started = False
        self.is_shutdown_caller = False
        self.coverage = None
        self.arrived_points = 0
        self.explore_id = None
        self.exploration_complete = False
        self.still_processing = False
        self.base_station_started = False
        self.robot_count = rospy.get_param("~robot_count")
        self.environment = rospy.get_param("~environment")
        self.debug_mode = rospy.get_param("~debug_mode")
        self.termination_metric = rospy.get_param("~termination_metric")
        self.share_limit = rospy.get_param('~data_share_threshold')
        self.target_distance = rospy.get_param('~target_distance')
        self.target_angle = rospy.get_param('~target_angle')
        self.run = rospy.get_param("~run")
        self.max_exploration_time = rospy.get_param("~max_exploration_time")
        self.max_coverage = rospy.get_param("~max_coverage")
        self.max_common_coverage = rospy.get_param("~max_common_coverage")
        self.bs_pose = rospy.get_param("~bs_pose")
        self.graph_scale = rospy.get_param("~graph_scale")
        self.max_target_info_ratio = rospy.get_param("~max_target_info_ratio")
        self.conn_manager = {}
        self.exploration_time = rospy.Time.now().to_sec()
        self.candidate_robots = self.relay_robots + self.base_stations

        rospy.Service("/robot_{}/home_alert".format(self.robot_id), HomeAlert, self.home_alert_callback)
        rospy.Service("/robot_{}/shared_data".format(self.robot_id), SharedData, self.shared_data_handler)
        self.auction_points_srv = rospy.Service("/robot_{}/auction_points".format(self.robot_id), SharedPoint,
                                                self.shared_point_handler)
        self.alloc_point_srv = rospy.Service("/robot_{}/allocated_point".format(self.robot_id), SharedFrontier,
                                             self.shared_frontier_handler)
        rospy.Subscriber('/robot_{}/initial_data'.format(self.robot_id), BufferedData, self.buffered_data_callback,
                         queue_size=100)
        self.data_size_pub = rospy.Publisher('/shared_data_size', DataSize, queue_size=10)
        self.signal_strength_srv = rospy.ServiceProxy("/signal_strength".format(self.robot_id), HotSpot)
        self.fetch_frontier_points = rospy.ServiceProxy('/robot_{}/frontier_points'.format(self.robot_id),
                                                        FrontierPoint)
        self.fetch_rendezvous_points = rospy.ServiceProxy('/robot_{}/rendezvous'.format(self.robot_id),
                                                          RendezvousPoints)

        self.karto_pub = rospy.Publisher('/robot_{}/karto_in'.format(self.robot_id), LocalizedScan, queue_size=1000)

        for ri in self.candidate_robots:
            pub = rospy.Publisher("/robot_{}/initial_data".format(ri), BufferedData, queue_size=1000)
            pub1 = rospy.Publisher("/robot_{}/rendezvous_points".format(ri), RendezvousLocations, queue_size=10)
            alert_pub = rospy.ServiceProxy("/robot_{}/home_alert".format(ri), HomeAlert)
            received_data_clt = rospy.ServiceProxy("/robot_{}/shared_data".format(ri), SharedData)
            action_points_clt = rospy.ServiceProxy("/robot_{}/auction_points".format(ri), SharedPoint)
            alloc_point_clt = rospy.ServiceProxy("/robot_{}/allocated_point".format(ri), SharedFrontier)
            self.conn_manager[ri] = rospy.Time.now().to_sec()

            self.publisher_map[ri] = pub
            self.rendezvous_publishers[ri] = pub1
            self.home_alert_map[ri] = alert_pub
            self.shared_data_srv_map[ri] = received_data_clt
            self.shared_point_srv_map[ri] = action_points_clt
            self.allocation_srv_map[ri] = alloc_point_clt

        if robot_type == RR_TYPE:
            self.parent_robot_id = self.base_stations[0]
            # =========   Navigation and exploration =======
            rospy.Subscriber("/robot_{}/MoveTo/status".format(self.robot_id), GoalStatusArray,
                             self.move_status_callback)
            rospy.Subscriber("/robot_{}/MoveTo/result".format(self.robot_id), MoveToPosition2DActionResult,
                             self.move_result_callback)
            self.moveTo_pub = rospy.Publisher("/robot_{}/MoveTo/goal".format(self.robot_id), MoveToPosition2DActionGoal,
                                              queue_size=10)
            self.start_exploration = rospy.ServiceProxy('/robot_{}/StartExploration'.format(self.robot_id), Trigger)
            rospy.Subscriber('/robot_{}/Explore/status'.format(self.robot_id), GoalStatusArray,
                             self.exploration_callback)
            self.cancel_explore_pub = rospy.Publisher("/robot_{}/Explore/cancel".format(self.robot_id), GoalID,
                                                      queue_size=10)
            self.start_exploration_pub = rospy.Publisher("/robot_{}/Explore/goal".format(self.robot_id),
                                                         ExploreActionGoal, queue_size=10)
            rospy.Subscriber('/robot_{}/Explore/result'.format(self.robot_id), ExploreActionResult,
                             self.exploration_result_callback)
            self.move_to_stop = rospy.ServiceProxy('/robot_{}/Stop'.format(self.robot_id), Trigger)
            self.pose_publisher = rospy.Publisher("/robot_{}/cmd_vel".format(self.robot_id), Twist, queue_size=1)
            rospy.Subscriber("/robot_{}/pose".format(self.robot_id), Pose, callback=self.pose_callback)
            # ============ Ends Here ======================

            rospy.Subscriber('/robot_{}/rendezvous_points'.format(self.robot_id), RendezvousLocations,
                             self.callback_rendezvous_points)
            self.chose_point_pub = rospy.Publisher("/chosen_point", ChosenPoint, queue_size=10)
            rospy.Subscriber("/chosen_point", ChosenPoint, self.chosen_point_callback)

        self.explored_region = rospy.ServiceProxy("/robot_{}/explored_region".format(self.robot_id), ExploredRegion)
        rospy.Subscriber('/karto_out', LocalizedScan, self.robots_karto_out_callback, queue_size=10)
        for i in self.candidate_robots + [self.robot_id]:
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
        # ======= pose transformations====================
        self.listener = tf.TransformListener()

        rospy.loginfo("Robot {} Initialized successfully!!".format(self.robot_id))

    def spin(self):
        r = rospy.Rate(0.05)
        while not rospy.is_shutdown():
            try:
                self.update_base_points()
                pu.log_msg(self.robot_id, "Is exploring: {}".format(self.is_exploring), self.debug_mode)
                if self.is_exploring:
                    self.check_if_time_to_getback_to_rendezvous()
            except Exception as e:
                pu.log_msg(self.robot_id, "Got Error: {}".format(e), self.debug_mode)
            r.sleep()

    def is_time_to_share(self, rid):
        now = rospy.Time.now().to_sec()
        diff = now - self.conn_manager[rid]
        return diff > self.share_limit

    def update_share_time(self, rid):
        self.conn_manager[rid] = rospy.Time.now().to_sec()

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

    ''' This publishes the newly computed rendezvous points for 10 seconds in an interval of 1 second'''

    def publish_rendezvous_points(self, rendezvous_poses, receivers, direction=1, total_exploration_time=0):
        for i in range(len(receivers)):
            id = receivers[i]
            rv_points = RendezvousLocations()
            rv_points.poses = [rendezvous_poses[i]]
            rv_points.msg_header.header.frame_id = '{}'.format(self.robot_id)
            rv_points.msg_header.sender_id = str(self.robot_id)
            rv_points.msg_header.topic = 'rendezvous_points'
            rv_points.exploration_time = total_exploration_time
            rv_points.direction = direction
            rv_points.msg_header.receiver_id = str(id)
            self.rendezvous_publishers[id].publish(rv_points)

    def robots_karto_out_callback(self, data):
        if data.robot_id - 1 == self.robot_id:
            for rid in self.candidate_robots:
                self.add_to_file(rid, [data])
            if self.robot_type == RR_TYPE and self.is_initial_rendezvous_sharing:
                self.is_initial_rendezvous_sharing = False
                self.push_messages_to_receiver([self.parent_robot_id])

    def start_exploration_action(self, new_point, direction=0):
        if direction == TO_RENDEZVOUS:
            self.rendezvous_point = copy.deepcopy(new_point)
            self.move_robot_to_goal(self.rendezvous_point, TO_RENDEZVOUS)
            pu.log_msg(self.robot_id, "Going to RV point: {}".format(self.rendezvous_point), self.debug_mode)
        elif direction == TO_FRONTIER:
            if not self.frontier_point:
                self.previous_frontier_point = copy.deepcopy(self.frontier_point)
            self.frontier_point = new_point
            self.move_robot_to_goal(self.frontier_point, TO_FRONTIER)
            pu.log_msg(self.robot_id, "Going to frontier: {}".format(self.frontier_point), self.debug_mode)

    def map_callback(self, data):
        self.last_map_update_time = rospy.Time.now().to_sec()

    def update_base_points(self):
        if not self.still_processing:
            self.still_processing = True
            self.get_explored_region()
            self.compute_map_points()
            self.still_processing = False

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

    def parse_rendezvous_locations(self, poses):
        points = []
        for p in poses:
            point = (p.position.x, p.position.y)
            point = pu.scale_down(point, self.graph_scale)
            points.append(point)
        return points

    def exploration_callback(self, data):
        if data.status_list:
            goal_status = data.status_list[0]
            gid = goal_status.goal_id.id
            if gid and self.exploration_id != gid:
                self.exploration_id = gid

    def exploration_result_callback(self, data):
        if data.status.status == SUCCEEDED:
            pu.log_msg(self.robot_id, "Received exploration result: {}".format(data.status.status), self.debug_mode)
            self.exploration_complete = True
            self.move_back_to_base_station()

    def get_close_devices(self):
        robots = []
        devices = []
        try:
            ss_data = self.signal_strength_srv(HotSpotRequest(robot_id=str(self.robot_id)))
            data = ss_data.hot_spots
            signals = data.signals
            for rs in signals:
                robots.append([rs.robot_id, rs.rssi])
                devices.append(str(rs.robot_id))
        except:
            pass
        return set(devices)

    def callback_rendezvous_points(self, data):
        sender_id = data.msg_header.header.frame_id
        if sender_id == str(self.parent_robot_id):
            received_points = self.parse_rendezvous_locations(data.poses)
            self.wait_for_map_update()
            self.move_attempt = 0
            robot_pose = self.get_robot_pose()
            optimal_points = self.get_available_points(received_points)
            new_point = self.get_closest_point(robot_pose, self.rendezvous_points)
            if new_point:
                self.rendezvous_point = copy.deepcopy(new_point)
            else:
                self.rendezvous_point = robot_pose
            self.rendezvous_points = optimal_points
            self.move_robot_to_goal(self.rendezvous_point, TO_RENDEZVOUS)

    '''This is executed by the BS and FR to determine whether its time to move back to a rendezvous location and 
    share information '''

    def check_if_time_to_getback_to_rendezvous(self):
        pu.log_msg(self.robot_id, "TargetInfoRatio: {},{},{}".format(self.target_info_ratio, len(self.base_points),
                                                                     len(self.robot_points)), 1)
        if self.target_info_ratio >= self.max_target_info_ratio:
            self.move_back_to_base_station()
        else:
            pu.log_msg(self.robot_id, "Target ratio: {}".format(self.target_info_ratio), self.debug_mode)

    def process_alert_data(self, received_data):
        for sender_id, data in received_data.items():
            received_buff_data = data.res_data
            sender_id = received_buff_data.msg_header.header.frame_id
            heading_back = received_buff_data.heading_back
            if heading_back:
                if self.heading_back:
                    self.heading_back = 0  # then you've shared your data and you're all set. continue with exploration
                    self.base_points = copy.deepcopy(self.robot_points)
            thread = Thread(target=self.process_data, args=(sender_id, received_buff_data,))
            thread.start()
            pu.log_msg(self.robot_id, "Processed alert data", self.debug_mode)

    def process_parent_data(self, data, sent_data=[]):
        received_buff_data = data.res_data
        sender_id = received_buff_data.msg_header.header.frame_id
        thread = Thread(target=self.process_data, args=(sender_id, received_buff_data,))
        thread.start()
        self.base_points = copy.deepcopy(self.robot_points)  # received_buff_data.base_map
        data_size = self.get_data_size(received_buff_data.data) + self.get_data_size(sent_data)
        self.report_shared_data(data_size)
        self.compute_map_points()
        pu.log_msg(self.robot_id, "Processed parent data", self.debug_mode)
        if self.exploration_complete:
            pu.log_msg(self.robot_id, "Exploration complete!!", self.debug_mode)
        else:
            response = self.fetch_frontier_points(FrontierPointRequest(count=len(self.candidate_robots) + 1))
            ridges = response.ridges
            poses = [r.nodes[1] for r in ridges]
            received_points = self.parse_rendezvous_locations(poses)
            self.move_attempt = 0
            if received_points:
                pu.log_msg(self.robot_id, "Fetched new frontier points", self.debug_mode)
                new_point = received_points[0]
                self.start_exploration_action(new_point=new_point, direction=TO_FRONTIER)
            else:
                pu.log_msg(self.robot_id, "Computed frontier points but returned none", self.debug_mode)

    ''' Helper method to publish a message to any receiver karto_in topic'''

    def push_messages_to_receiver(self, receiver_ids, is_alert=0):
        for rid in receiver_ids:
            message_data = self.load_data_for_id(rid)
            buffered_data = BufferedData()
            buffered_data.msg_header.header.stamp.secs = rospy.Time.now().secs
            buffered_data.msg_header.header.frame_id = '{}'.format(self.robot_id)
            buffered_data.msg_header.sender_id = str(self.robot_id)
            buffered_data.msg_header.receiver_id = str(rid)
            buffered_data.msg_header.topic = 'initial_data'
            buffered_data.secs = []
            buffered_data.data = message_data
            buffered_data.base_map = []
            if self.robot_type == BS_TYPE:
                points = list(self.base_points)
                buffered_data.base_map = self.create_poses(points)
            buffered_data.heading_back = self.heading_back
            buffered_data.alert_flag = is_alert
            self.publisher_map[rid].publish(buffered_data)
            self.delete_data_for_id(rid)

    def create_poses(self, points):
        poses = []
        for p in points:
            pose = Pose()
            pose.position.x = p[pu.INDEX_FOR_X]
            pose.position.y = p[pu.INDEX_FOR_Y]
            poses.append(pose)
        return poses

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
        self.robot_stopped = False
        self.chose_point_pub.publish(chosen_point)
        self.robot_state = PASSIVE_STATE

    def move_back_to_base_station(self):
        self.cancel_exploration()
        self.heading_back = 1
        self.is_exploring = False
        robot_pose = self.get_robot_pose()
        pu.log_msg(self.robot_id, "Heading back to RV point: {}".format(self.rendezvous_point), self.debug_mode)
        point = self.get_closest_point(robot_pose, self.rendezvous_points)
        if point:
            self.rendezvous_point = point
        self.move_robot_to_goal(self.rendezvous_point, FROM_EXPLORATION)

    def get_closest_point(self, pose, points):
        dists = {}
        chosen_point = None
        for p in points:
            dists[pu.D(pose, p)] = p
        if dists:
            chosen_point = dists[min(dists.keys())]
        return chosen_point

    ''' This is a call back that receives the status of the robot during navigation'''

    def move_status_callback(self, data):
        id_4 = "robot_{}_{}_{}".format(self.robot_id, self.goal_count - 1, FROM_EXPLORATION)
        id_1 = "robot_{}_{}_{}".format(self.robot_id, self.goal_count - 1, TO_RENDEZVOUS)
        id_0 = "robot_{}_{}_{}".format(self.robot_id, self.goal_count - 1, TO_FRONTIER)
        if data.status_list:
            goal_status = data.status_list[0]
            if goal_status.goal_id.id:
                if goal_status.goal_id.id == id_4 or goal_status.goal_id.id == id_1:
                    if not self.robot_stopped and goal_status.status == ACTIVE or goal_status.status == ABORTED:
                        close_devices = self.get_close_devices()
                        if self.parent_robot_id in close_devices:
                            self.move_to_stop()
                            self.robot_stopped = True
                            pose = self.get_robot_pose()
                            self.rendezvous_point = pose
                            if goal_status.goal_id.id == id_1:
                                self.push_messages_to_receiver([self.parent_robot_id])
                            else:
                                pu.log_msg(self.robot_id, "Back from exploration.. stopped: there's connectivity",
                                           self.debug_mode)
                                self.share_data_with_parent()
                        else:
                            if goal_status.goal_id.id == id_4 and close_devices:
                                received_data = {}
                                data_size = 0
                                for rid in close_devices:
                                    receiver_id = str(rid)
                                    if self.is_time_to_share(receiver_id):
                                        self.update_share_time(receiver_id)
                                        buff_data = self.create_buff_data(rid)
                                        try:
                                            home_alert_resp = self.home_alert_map[rid](
                                                HomeAlertRequest(robot_id=self.robot_id, home_alert=1,
                                                                 req_data=buff_data))
                                            received_data[receiver_id] = home_alert_resp.res_data
                                            data_size += self.get_data_size(home_alert_resp.res_data.data)
                                            self.delete_data_for_id(rid)
                                        except Exception as e:
                                            pu.log_msg(self.robot_id, "Error sending data to: {}".format(receiver_id),
                                                       self.debug_mode)
                                        data_size += self.get_data_size(buff_data.data)
                                if data_size:
                                    self.report_shared_data(data_size)
                                    self.process_alert_data(received_data)
                                    if not self.heading_back:
                                        self.move_to_stop()
                                        self.initiate_exploration()

                elif goal_status.goal_id.id == id_0:
                    if goal_status.status == ACTIVE:
                        if goal_status.status == ABORTED:
                            pu.log_msg(self.robot_id, "Couldn't reach goal...Starting exploration now", self.debug_mode)
                            self.initiate_exploration()

    '''This callback receives the final result of the most recent navigation to the goal'''

    def move_result_callback(self, data):
        id_4 = "robot_{}_{}_{}".format(self.robot_id, self.goal_count - 1, FROM_EXPLORATION)
        id_1 = "robot_{}_{}_{}".format(self.robot_id, self.goal_count - 1, TO_RENDEZVOUS)
        id_0 = "robot_{}_{}_{}".format(self.robot_id, self.goal_count - 1, TO_FRONTIER)
        if data.status:
            if data.status.status == ABORTED:
                if data.status.goal_id.id == id_1:
                    if self.move_attempt < MAX_ATTEMPTS:
                        pu.log_msg(self.robot_id, "Navigation failed. Trying again..", self.debug_mode)
                        self.rotate_robot()
                        self.move_robot_to_goal(self.rendezvous_point, TO_RENDEZVOUS)
                        self.move_attempt += 1
                    else:
                        pu.log_msg(self.robot_id, "To rendezvous Navigation failed!", self.debug_mode)
                        self.failed_points.append(self.rendezvous_point)
                        rv_ps = [p for p in self.rendezvous_points if p not in self.failed_points]
                        if rv_ps:
                            self.rendezvous_point = rv_ps[-1]
                            self.move_robot_to_goal(self.rendezvous_point, TO_RENDEZVOUS)
                        else:
                            pu.log_msg(self.robot_id, "To rendezvous arrived!", self.debug_mode)
                            self.push_messages_to_receiver([self.parent_robot_id])

                elif data.status.goal_id.id == id_0:
                    pu.log_msg(self.robot_id, "To arrived at frontier!!", self.debug_mode)
                    self.initiate_exploration()

                elif data.status.goal_id.id == id_4:
                    self.heading_back = 0
                    self.is_exploring = True
                    self.move_to_stop()
                    self.initiate_exploration()

            elif data.status.status == SUCCEEDED:
                self.retry_attempts = 0
                if data.status.goal_id.id == id_1:
                    pu.log_msg(self.robot_id, "Arrived at RV point: {}".format(self.rendezvous_point), self.debug_mode)
                    self.heading_back = 0
                    self.push_messages_to_receiver([self.parent_robot_id])

                elif data.status.goal_id.id == id_4:  # to meeting point for FRs
                    pu.log_msg(self.robot_id, "Back from exploration..", self.debug_mode)
                    self.move_to_stop()
                    self.share_data_with_parent()

                elif data.status.goal_id.id == id_0:
                    pu.log_msg(self.robot_id, "Arrived at Frontier: {}".format(self.frontier_point), self.debug_mode)
                    self.initiate_exploration()

    def get_data_size(self, buff_data):
        data_size = len(buff_data)  # sys.getsizeof(buff_data)
        return data_size

    def report_shared_data(self, shared_size):
        data_size = DataSize()
        data_size.header.frame_id = '{}'.format(self.robot_id)
        data_size.header.stamp = rospy.Time.now()
        data_size.size = shared_size
        data_size.session_id = 'recurrent'
        self.data_size_pub.publish(data_size)

    def initiate_exploration(self):
        result = self.start_exploration()
        if result:
            pu.log_msg(self.robot_id, "Starting Exploration ...", self.debug_mode)
            self.exploration_start_time = rospy.Time.now()
            self.is_exploring = True
            self.heading_back = 0
            self.robot_state = ACTIVE_STATE
        else:
            pu.log_msg(self.robot_id, "Failed to start exploration...", self.debug_mode)

    def share_data_with_parent(self):
        parent_data = self.create_buff_data(self.parent_robot_id)
        new_data = self.shared_data_srv_map[self.parent_robot_id](SharedDataRequest(req_data=parent_data))
        self.process_parent_data(new_data)
        pu.log_msg(self.robot_id, "received a response", self.debug_mode)
        self.delete_data_for_id(self.parent_robot_id)

    def process_data(self, sender_id, buff_data):
        # self.lock.acquire()
        data_vals = buff_data.data
        for scan in data_vals:
            self.karto_pub.publish(scan)
        for rid in self.candidate_robots:
            if rid != sender_id:
                self.add_to_file(rid, data_vals)
        # self.lock.release()

    def shared_data_handler(self, data):
        pu.log_msg(self.robot_id, "Received request from robot..", self.debug_mode)
        received_buff_data = data.req_data
        sender_id = received_buff_data.msg_header.header.frame_id
        self.update_share_time(sender_id)
        buff_data = self.create_buff_data(sender_id, is_alert=0)
        thread = Thread(target=self.process_data, args=(sender_id, received_buff_data,))
        thread.start()
        self.delete_data_for_id(sender_id)
        pu.log_msg(self.robot_id, "Processed received data..", self.debug_mode)
        return SharedDataResponse(poses=[], res_data=buff_data)

    def create_buff_data(self, rid, is_alert=0, is_home_alert=False):
        message_data = self.load_data_for_id(rid)
        if is_home_alert:
            message_data = message_data + self.load_data_for_id(self.parent_robot_id)

        buffered_data = BufferedData()
        buffered_data.msg_header.header.stamp.secs = rospy.Time.now().secs
        buffered_data.msg_header.header.frame_id = '{}'.format(self.robot_id)
        buffered_data.msg_header.sender_id = str(self.robot_id)
        buffered_data.msg_header.receiver_id = str(rid)
        buffered_data.msg_header.topic = 'initial_data'
        buffered_data.secs = []
        buffered_data.data = copy.deepcopy(message_data)
        if self.robot_type == BS_TYPE:
            points = list(self.base_points)
            buffered_data.base_map = self.create_poses(points)
        buffered_data.heading_back = self.heading_back
        buffered_data.alert_flag = is_alert
        return buffered_data

    '''This callback receives the incoming karto_in data'''

    def buffered_data_callback(self, buff_data):
        sender_id = buff_data.msg_header.header.frame_id
        if sender_id in self.candidate_robots:
            self.process_data(sender_id, buff_data)
            pu.log_msg(self.robot_id, "Received data from: {}".format(sender_id), self.debug_mode)
            if self.robot_type == BS_TYPE:
                self.push_messages_to_receiver([sender_id])
                direction = TO_RENDEZVOUS
                if not self.base_station_started:
                    self.initial_data_count += 1
                    pu.log_msg(self.robot_id, "Initial data count {}".format(self.initial_data_count), self.debug_mode)
                    if self.initial_data_count == len(self.candidate_robots):
                        self.base_station_started = True
                        self.is_initial_rendezvous_sharing = False
                        pu.log_msg(self.robot_id, "Received data all the data", self.debug_mode)
                        pose_v = self.get_robot_pose()
                        p = Pose()
                        p.position.x = pose_v[pu.INDEX_FOR_X]
                        p.position.y = pose_v[pu.INDEX_FOR_Y]
                        rendezvous_points = self.fetch_rendezvous_points(
                            RendezvousPointsRequest(count=len(self.candidate_robots), pose=p))
                        rv_poses = rendezvous_points.poses
                        if rv_poses:
                            self.publish_rendezvous_points(rv_poses, self.candidate_robots, direction=direction)
                            self.arrived_points = 0
                        else:
                            pu.log_msg(self.robot_id, "Robot {}: No rendzvous points to send...".format(self.robot_id),
                                       self.debug_mode)
                else:
                    self.arrived_points += 1
                    if self.arrived_points == len(self.candidate_robots):
                        pu.log_msg(self.robot_id, "Computing frontier points for all", self.debug_mode)
                        self.request_and_share_frontiers()
            else:
                if sender_id == self.parent_robot_id:
                    self.base_points = self.base_points.union(
                        {(p.position.x, p.position.y) for p in buff_data.base_map})

    def home_alert_callback(self, data):
        pu.log_msg(self.robot_id, 'Home Alert received from {}'.format(data.robot_id), self.debug_mode)
        received_data = data.req_data
        sender_id = received_data.msg_header.header.frame_id
        thread = Thread(target=self.process_data, args=(sender_id, received_data,))
        thread.start()
        buff_data = self.create_buff_data(sender_id, is_home_alert=1)
        self.delete_data_for_id(sender_id)
        return HomeAlertResponse(res_data=buff_data)

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

    def compute_map_points(self):
        self.get_explored_region()
        common_region = self.robot_points.intersection(self.base_points)
        self.info_base = float(len(self.base_points))
        self.new_info = len(self.robot_points) - len(common_region)
        if self.info_base or self.new_info:
            self.target_info_ratio = 1 - (self.info_base / float(self.new_info + self.info_base))

    def get_explored_region(self):
        try:
            explored_points = self.explored_region(ExploredRegionRequest(robot_id=self.robot_id))
            poses = explored_points.poses
            for p in poses:
                if self.robot_type == BS_TYPE:
                    self.base_points.add((p.position.x, p.position.y))
                else:
                    self.robot_points.add((p.position.x, p.position.y))
        except Exception as e:
            pass

    def round_point(self, p):
        xc = round(p[pu.INDEX_FOR_X], 2)
        yc = round(p[pu.INDEX_FOR_Y], 2)
        new_p = [0.0] * 2
        new_p[pu.INDEX_FOR_X] = xc
        new_p[pu.INDEX_FOR_Y] = yc
        new_p = tuple(new_p)
        return new_p

    def pose_callback(self, msg):
        pose = (msg.x, msg.y, msg.theta)
        self.robot_pose = pose

    def cancel_exploration(self):
        if self.exploration_id:
            rospy.logerr("Robot {} Cancelling exploration...".format(self.robot_id))
            goal_id = GoalID()
            goal_id.id = self.exploration_id
            self.cancel_explore_pub.publish(goal_id)
        else:
            rospy.logerr("Exploration ID not set...")

    def get_elevation(self, quaternion):
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        return yaw

    def parse_frontier_response(self, data):
        frontier_points = {}
        received_ridges = data.ridges
        for r in received_ridges:
            p = r.nodes[1]
            frontier_points[(p.position.x, p.position.y)] = r
        return frontier_points

    def request_and_share_frontiers(self, direction=0):
        frontier_point_response = self.fetch_frontier_points(FrontierPointRequest(count=len(self.candidate_robots) + 1))
        frontier_points = self.parse_frontier_response(frontier_point_response)
        if frontier_points:
            auction = self.create_auction(frontier_points)
            auction_feedback = {}
            for rid in self.candidate_robots:
                auction_response = self.shared_point_srv_map[rid](SharedPointRequest(req_data=auction))
                data = auction_response.res_data
                self.all_feedbacks[rid] = data
                m = len(data.distances)
                min_dist = max(data.distances)
                min_pose = None
                for i in range(m):
                    if min_dist >= data.distances[i]:
                        min_pose = data.poses[i]
                        min_dist = data.distances[i]
                auction_feedback[rid] = (min_dist, min_pose)
                self.compute_and_share_auction_points(auction_feedback, frontier_points)

            pu.log_msg(self.robot_id, "Point allocation complete", self.debug_mode)
            self.all_feedbacks.clear()

    def create_auction(self, rendezvous_poses, distances=[]):
        auction = Auction()
        auction.msg_header.header.frame_id = '{}'.format(self.robot_id)
        auction.msg_header.sender_id = str(self.robot_id)
        auction.msg_header.topic = 'auction_points'
        auction.msg_header.header.stamp = rospy.Time.now()
        auction.session_id = ''
        auction.distances = distances
        auction.poses = []
        self.all_feedbacks.clear()
        for k, v in rendezvous_poses.items():
            pose = Pose()
            pose.position.x = k[pu.INDEX_FOR_X]
            pose.position.y = k[pu.INDEX_FOR_Y]
            auction.poses.append(pose)
        return auction

    def create_frontier(self, receiver, ridge):
        frontier = Frontier()
        frontier.msg_header.header.frame_id = '{}'.format(self.robot_id)
        frontier.msg_header.header.stamp = rospy.Time.now()
        frontier.msg_header.sender_id = str(self.robot_id)
        frontier.msg_header.receiver_id = str(receiver)
        frontier.msg_header.topic = 'allocated_point'
        frontier.ridge = ridge  #
        frontier.session_id = ''
        return frontier

    def compute_and_share_auction_points(self, auction_feedback, frontier_points):
        all_robots = list(auction_feedback)
        taken_poses = []
        for i in all_robots:
            pose_i = (auction_feedback[i][1].position.x, auction_feedback[i][1].position.y)
            conflicts = []
            for j in all_robots:
                if i != j:
                    pose_j = (auction_feedback[j][1].position.x, auction_feedback[j][1].position.y)
                    if pose_i == pose_j:
                        conflicts.append((i, j))
            rpose = auction_feedback[i][1]
            frontier = self.create_frontier(i, frontier_points[(rpose.position.x, rpose.position.y)])
            pu.log_msg(self.robot_id, "Sharing point with {}".format(i), self.debug_mode)
            res = self.allocation_srv_map[i](SharedFrontierRequest(frontier=frontier))
            pu.log_msg(self.robot_id, "Shared a frontier point with robot {}: {}".format(i, res), self.debug_mode)
            taken_poses.append(pose_i)
            for c in conflicts:
                conflicting_robot_id = c[1]
                feedback_for_j = self.all_feedbacks[conflicting_robot_id]
                rob_poses = feedback_for_j.poses
                rob_distances = feedback_for_j.distances
                remaining_poses = {}
                for k in range(len(rob_poses)):
                    p = (rob_poses[k].position.x, rob_poses[k].position.y)
                    if p != pose_i and p not in taken_poses:
                        remaining_poses[rob_distances[k]] = rob_poses[k]
                next_closest_dist = min(list(remaining_poses))
                auction_feedback[conflicting_robot_id] = (next_closest_dist, remaining_poses[next_closest_dist])
        return taken_poses

    def shared_frontier_handler(self, req):
        data = req.frontier
        pu.log_msg(self.robot_id, "Received new frontier point", self.debug_mode)
        self.frontier_ridge = data.ridge
        new_point = [0.0] * 2
        new_point[pu.INDEX_FOR_X] = self.frontier_ridge.nodes[1].position.x
        new_point[pu.INDEX_FOR_Y] = self.frontier_ridge.nodes[1].position.y
        new_point = pu.scale_down(new_point, self.graph_scale)
        self.frontier_point = new_point
        robot_pose = self.get_robot_pose()
        self.frontier_data.append(
            {'time': rospy.Time.now().to_sec(), 'distance_to_frontier': pu.D(robot_pose, new_point)})
        self.move_robot_to_goal(self.frontier_point, TO_FRONTIER)
        pu.log_msg(self.robot_id, "Received allocated points", self.debug_mode)
        return SharedFrontierResponse(success=1)

    def shared_point_handler(self, auction_data):
        pu.log_msg(self.robot_id, "Received auction", self.debug_mode)
        data = auction_data.req_data
        sender_id = data.msg_header.header.frame_id
        poses = data.poses
        if not poses:
            pu.log_msg(self.robot_id, "No poses received. Proceeding to my next frontier", self.debug_mode)
            self.start_exploration_action(self.frontier_point)
            return SharedPointResponse(auction_accepted=1, res_data=None)
        received_points = []
        distances = []
        robot_pose = self.get_robot_pose()
        robot_pose = pu.scale_up(robot_pose, self.graph_scale)
        for p in poses:
            received_points.append(p)
            point = (p.position.x, p.position.y,
                     self.get_elevation((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)))
            distance = pu.D(robot_pose, point)
            distances.append(distance)
        auction = Auction()
        auction.msg_header.header.frame_id = '{}'.format(self.robot_id)
        auction.msg_header.header.stamp = rospy.Time.now()
        auction.msg_header.sender_id = str(self.robot_id)
        auction.msg_header.receiver_id = str(sender_id)
        auction.msg_header.topic = 'auction_feedback'
        auction.poses = received_points
        auction.distances = distances
        auction.session_id = data.session_id
        return SharedPointResponse(auction_accepted=1, res_data=auction)

    def add_to_file(self, rid, data):
        # self.lock.acquire()
        if rid in self.karto_messages:
            self.karto_messages[rid] += data
        else:
            self.karto_messages[rid] = data
        # self.lock.release()
        return True

    def load_data_for_id(self, rid):
        message_data = []
        if rid in self.karto_messages:
            message_data = self.karto_messages[rid]
        return message_data

    def load_data(self):
        return self.karto_messages

    def delete_data_for_id(self, rid):
        sleep(2)
        self.lock.acquire()
        if rid in self.karto_messages:
            del self.karto_messages[rid]
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

    def get_robot_pose(self):
        if self.robot_type == BS_TYPE:
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
                robot_pose = (math.floor(robot_loc_val[0]), math.floor(robot_loc_val[1]))
                sleep(1)
            except:
                pu.log_msg(self.robot_id, "Can't fetch robot pose from tf", self.debug_mode)
                pass
        return robot_pose

    def save_all_data(self):
        # self.graph_processor.save_all_data()
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
