#!/usr/bin/env python

import heapq
import tf
import math
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

class Point:
    def __init__(self, x, y, th=None):
        self.x = x
        self.y = y
        self.th = th

def get_distance(point1, point2):
    x = point1[0] - point2[0]
    y = point1[1] - point2[1]
    return (x**2 + y**2)**.5


def trace_path(node):
    reverse_path = []
    while node is not None:
        reverse_path.append(node[0])
        node = node[1]
    return reverse_path[::-1]

class Map:

    def __init__(self):
        self.graph = None
        self.probability_threshold = None # Point to probability dictionary
        self.probabilities = None
        self.x_bounds = None
        self.y_bounds = None
        self.origin = None
        self.resolution = None
        self.theta = None

    def __contains__(self, point):
        transformed = self.pose_to_map_pixel(point)
        return transformed in self.graph

    def get_neighbors(self, point):
        transformed = self.pose_to_map_pixel(point)
        if transformed in self.graph:
            return self.get_points_around(transformed)
        else:
            raise KeyError

    def get_points_around(self, point):
        o_x = point[0]
        o_y = point[1]
        valid_neighbors = set()
        for dx in range(-1,2,1):
            nx = o_x+dx
            for dy in range(-1, 2, 1):
                ny = o_y+dy
                npo = (nx, ny)
                if npo in self.graph:
                    if self.graph[npo] == 0:
                        valid_neighbors.add(self.map_pixel_to_pose(npo))
        if point in valid_neighbors:
            valid_neighbors.remove(point)
        return valid_neighbors


    def get_heuristic(self, point1, point2):
        x = point1[0] - point2[0]
        y = point1[1] - point2[1]
        return (x**2 + y**2)**.5

    def set_threshhold(self, threshhold):
        self.probability_threshold = threshhold

    def translate_occupancy_grid_msg_to_graph(self, msg):
        details = msg.info
        origin_x = details.origin.position.x
        origin_y = details.origin.position.y
        print("Origin:", origin_x, origin_y)
        # need to use quaternion to calc theta (rotation)
        rotation = (details.origin.orientation.x, details.origin.orientation.y,
            details.origin.orientation.z, details.origin.orientation.w)
        self.theta = tf.transformations.euler_from_quaternion(rotation)[2]
        probalilities_of_obstacles = msg.data
        self.probabilities = probalilities_of_obstacles
        resolution = details.resolution # float of m/cell
        self.resolution = resolution
        width = details.width # in cells
        height = details.height # in cells
        print(width, height)
        origin_of_map = details.origin # origin of map as pose (0, 0) represented by [m, m, rad]
        self.origin = origin_of_map
        self.x_bounds = width
        self.y_bounds = height
        self.graph = dict()
        for r in range(width):
            for c in range(height):
                relative_index = width * c + r
                # np = (int((math.cos(theta)*r - math.sin(theta)*c + origin_x)*resolution), int((math.sin(theta)*r + math.cos(theta)*c + origin_y)*resolution))
                # print(np)
                np = (r, c)
                self.graph[np] = self.probabilities[relative_index]

    def point_to_location(self, point):
        temp_point = Point(point[0]*self.resolution, point[0]*self.resolution)
        return temp_point

    def pose_to_map_pixel(self, pose):
        x = math.cos(self.theta)*pose[0] + math.sin(self.theta)*pose[1]
        y = math.cos(self.theta)*pose[1] - math.sin(self.theta)*pose[0]
        offset_x = math.cos(self.theta)*self.origin.position.x + math.sin(self.theta)*self.origin.position.y
        offset_y = math.cos(self.theta)*self.origin.position.y - math.sin(self.theta)*self.origin.position.x
        res = self.resolution
        px = int((x - offset_x)/res)
        py = int((y - offset_y)/res)
        return (px, py)

    def map_pixel_to_pose(self, pixel):
        res = self.resolution
        x = math.cos(self.theta)*pixel[0] - math.sin(self.theta)*pixel[1]
        y = math.cos(self.theta)*pixel[1] + math.sin(self.theta)*pixel[0]
        offset_x = self.origin.position.x
        offset_y = self.origin.position.y
        px = x*res + offset_x
        py = y*res + offset_y
        return (px, py)


def a_star(map, start, end, cost_func, heuristic):
    expanded = set()
    agenda = []
    heapq.heappush(agenda, (0, 0, (start, None)))
    print("start", start)
    print("goal", end)
    while agenda:
        _, cost, node = heapq.heappop(agenda)

        point = node[0]

        # print("publishing")
        # trajectory = LineTrajectory("/planned_trajectory")
        # trajectory.addPoint(Point(*point))
        # traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        # # publish trajectory
        # traj_pub.publish(trajectory.toPoseArray())
        # # visualize trajectory Markers
        # trajectory.publish_viz()

        # if we've already expanded this state, just move on; nothing to be
        # gained from doing it again...
        if point in expanded:
            continue

        # if we're done, return a path
        if end[0]-.1 <= point[0] <= end[0]+.1 and end[1]-.1 <= point[1] <= end[1]+.1:
            return trace_path(node)

        # otherwise, add this state to expanded, and add children to the
        # agenda.
        expanded.add(point)
        if point not in map:
            continue
        for child in map.get_neighbors(point):
            if child in expanded:
                # short-circuit here: if this child has already been expanded,
                # don't even bother putting it in the agenda.
                continue
            new_cost = cost + cost_func(point, child)
            priority = new_cost + heuristic(child)
            heapq.heappush(agenda, (priority, new_cost, (child, node)))

    # no more agenda; stop
    return None


class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        # CHANGE BACK BEFORE SUBMITTING
        # self.odom_topic = rospy.get_param("~odom_topic")
        self.odom_topic = "/odom"
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)

        self.map = None
        self.goal = None
        self.start = None

        self.odom_pub = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)

    def map_cb(self, msg):
        self.map = Map()
        self.map.set_threshhold(100)
        # this should take map msg and transform self.map to a set of nodes
        self.map.translate_occupancy_grid_msg_to_graph(msg)
        print("done making map")


    def odom_cb(self, msg):
        position = msg.pose.pose.position
        self.start = (position.x, position.y)


    def goal_cb(self, msg):
        position = msg.pose.position
        print("registered goal pose", self.map, self.start)
        self.goal = (position.x, position.y)
        if self.map is not None and self.goal is not None and self.start is not None:
            print("running plan_path")
            self.plan_path(self.start, self.goal, self.map)


    def plan_path(self, start_point, end_point, map):
        ## CODE FOR PATH PLANNING ##
        # self.trajectory must be in the map frame
        # print("running a star")
        # print("Start in map", self.start in self.map)
        # print("Start", self.start)
        # print("GOAL in MAP", self.goal in self.map)
        # print("Goal", self.goal)
        list_of_points = a_star(self.map, self.start, self.goal, cost_func=lambda n1, n2: get_distance(n1, n2), 
            heuristic=lambda node: self.map.get_heuristic(node, end_point))

        # print('Finished a star')
        for point in list_of_points:
            self.trajectory.addPoint(Point(*point))

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
