#!/usr/bin/env python2

import numpy as np
import rospy
import math
import heapq

from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped




class SafetyController:
    """
    Class to processes LaserScan data and output ackermann messages to prevent collisions
    """
    # ROS parameters
    SCAN_TOPIC = "/tesse/hood_lidar/scan"
    SAFETY_DRIVE_TOPIC = "/tesse/drive"
    DRIVE_COMMAND_TOPIC = "/vesc/high_level/ackermann_cmd_mux/output"

    # Scan Parameters
    SCAN_ANGLE_RANGE = math.pi/2.0 #Safety scan looks at forward 90 degrees

    # Global Thresholds
    Y_THRESH = .3
    X_THRESH = .3
    X_SCALE = .15
    

    def __init__(self):

        self.safety_publisher = rospy.Publisher(self.SAFETY_DRIVE_TOPIC,
                AckermannDriveStamped, queue_size=5)


        self.curr_steering_angle = 0
        self.curr_target_speed = 0
        self.recommended_speed = 0
        self.close_y_thresh = 0
        self.close_x_thresh = 0
        self.GAP_THRESHOLD = 1
        # Subscriber registration
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.final_safety_check)
        # rospy.Subscriber(self.DRIVE_COMMAND_TOPIC, AckermannDriveStamped, self.drive_callback)
	    # self.seq = 0

    def scan_callback(self, msg):
        """
        Processes the new scan data
        """
        # Convert scan data into 2D
        data = self.convert_to_cardinal(msg)

        # If the way ahead is blocked, stop
	self.danger_close(data)
        #Send out the message
        if self.recommended_speed < self.curr_target_speed:
            self.safety_publisher.publish(self.generate_stop_command())

    def drive_callback(self, msg):
        """
        Callback function
        Maintains the steering angle and desired speed of the vehicle according to controller.
        """
        self.curr_steering_angle = msg.drive.steering_angle
        self.curr_target_speed = msg.drive.speed
        self.close_y_thresh = self.Y_THRESH
        self.close_x_thresh = self.X_THRESH + self.X_SCALE * self.curr_target_speed

    def danger_close(self, xy_list):


        self.recommended_speed = self.curr_target_speed
        candidates = []
        # Holds all the points within y_threshold
        close_points = []
        # Holds all the points within x_threshold and y_threshold

        # find all points within horizontal range and filter out the
        # ones that are within a threshold of the vehicle
        for coord in xy_list:
            x, y = coord
            if abs(y) < self.close_y_thresh:
                candidates.append(coord)
                if x < self.close_x_thresh:
                    close_points.append(coord)
        if len(close_points) == 0:
            return

        # get the forward distance of the 10th percentile point
        np_close_points = np.array(close_points)
        avg_close_distance = np.percentile(np_close_points[:, 0], 10)


        # clip the speed to 0 if we are too close
        if(avg_close_distance < self.X_THRESH):
            self.recommended_speed = 0
        self.recommended_speed = max(self.recommended_speed, 0)

        print("distance: {}".format(avg_close_distance))
        print("speed: {}".format(self.recommended_speed))


    def path_ahead_is_blocked(self, xy_list):
        """
        Returns True if the path ahead is blocked based upon if the width of the region is greater than the threshold we set
        """

        gap = False
        first_point = True

        for coordinate_pair in xy_list:
            if gap:
                return False
                # Path is not blocked
            if first_point:
                prev_x = coordinate_pair[0]
                first_point = False
            elif abs(coordinate_pair[0] - prev_x) > self.GAP_THRESHOLD:
                gap = True
            else:
                prev_x = coordinate_pair[0]

        return not gap


    def generate_stop_command(self):
        """
        Creates an AckermannDriveStamped object
        Uses the value in self.curr_steering_angle to make angle
        adjustments.
        """
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = self.curr_steering_angle
        msg.drive.steering_angle_velocity = 0
        msg.drive.speed = self.recommended_speed
        msg.drive.acceleration = 0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        return msg


    def convert_to_cardinal(self, scan_msg):
        """
        Takes the scan_message and ouputs them into a list in 2D
        """
        cardinal_points= []
        curr_theta = scan_msg.angle_min
        # Remove all the data outside the following angle thesholds
        min_angle = -self.SCAN_ANGLE_RANGE / 2.0
        max_angle = self.SCAN_ANGLE_RANGE / 2.0
        for distance in scan_msg.ranges:
            if(curr_theta < min_angle):
                curr_theta += scan_msg.angle_increment
                continue
            elif(curr_theta > max_angle):
                break
            x = distance * math.cos(curr_theta)
            y = distance * math.sin(curr_theta)
            cardinal_points.append([x, y])
            curr_theta += scan_msg.angle_increment
        return cardinal_points

    def dist_to_wall(self, x, y, scan):     
        smolest = np.argpartition(scan, 3)
        avg_smol_distance = np.sum(scan[smolest[:3]]) / 3.0
        # l = list(scan)
        # heapq.heapify(l)
        # h = heapq.nsmallest(5, enumerate(l))
        # print("scan pt2:", scan)
        # avg_smol_distance = sum(list(h))/len(list(h))
        # print("avg: ",avg_smol_distance)
        return avg_smol_distance

    def final_safety_check(self, data):
        '''
        Safety controller for final challenge: plan is to avoid obstacles (ideally without stopping/slowing) using sensor equipment
        Goal: follow path of least resistance
        Idea: split lidar scan into sections, if forward section has close wall then check others and find path with farthest obstacles
        '''

        # Inits:
        pi = math.pi
        max_steering_angle = 1.0
        current_speed = 1.0  # Get speed from car, was 1
        dist_thresh = 3.5  # meters?

        angmin = data.angle_min
        angmax = data.angle_max
        anginc = data.angle_increment
        angles = np.array([data.angle_min + (i * data.angle_increment) for i in
                           range(0, int((data.angle_max - data.angle_min) / data.angle_increment))]) # was + 1 on the end range
        ranges = np.array(data.ranges)
        cosine = np.cos(angles)
        sine = np.sin(angles)
        x_coord = np.multiply(ranges, cosine)
        y_coord = np.multiply(ranges, sine)

        # First: check the scan data from the positive x-direction for a wall/obstacle closer than the threshold
        n = len(ranges)

        x_front = x_coord[2 * n // 5:3 * n // 5]
        y_front = y_coord[2 * n // 5:3 * n // 5]
        scan_front = ranges[6 * n // 15:10* n // 15]

        # Find distance from current position to nearest wall in front of car
        dist = self.dist_to_wall(x_front, y_front, scan_front)

        if dist > dist_thresh:
            return

        # If the distance to the wall we are approaching is too small, publish a steering angle
        # Find the distance to walls from other other sections
        sec_dists = []
        for i in [0, 1, 2, 4, 5, 6]:
            sec_x = x_coord[i * n // 7:(i + 1) * n // 7]
            sec_y = y_coord[i * n // 7:(i + 1) * n // 7]
            sec_ranges = ranges[i * n // 7:(i + 1) * n // 7]

            # Input values into distance finder, and add to our list of
            sec_dist = self.dist_to_wall(sec_x, sec_y, sec_ranges)
            sec_dists.append(sec_dist)

        # Goal: Find the section with the furthest wall, closest to "straight forwards"
        # Subtract threshold from distance array, take max difference and set appropriate steering angle
        diffs = np.array(sec_dists) - dist_thresh
        max_ind = np.argmax(diffs)

        # Indices 0-2 correspond to negative turning angles, while indices 3-5 are positive
        # Subtract 2.5 from argmax ind and normalize s.t. we get our desired fraction of the maximum turning angle that we want to turn
        norm_turn = (max_ind - 2.5) / 3

        angle_to_publish = norm_turn * max_steering_angle

        ################

        theta = np.pi / 2

        x_front = x_coord[:2 * n // 5]
        y_front = y_coord[:2 * n // 5]
        scan_left = ranges[5*n//15:8 * n // 15]
        d1 = self.dist_to_wall(x_front, y_front, scan_left)

        x_front = x_coord[4 * n // 5:]
        y_front = y_coord[4 * n // 5:]
        scan_right = ranges[8 * n // 15:11 *n //15]
        d2 = self.dist_to_wall(x_front, y_front, scan_right)

        if d1 < d2:
            theta = -1.0 * theta

        angle_to_publish = theta

        ##################


        # Publish steering command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.get_rostime()
        drive_msg.header.frame_id = "base_link_gt"
        drive_msg.drive.steering_angle = angle_to_publish
        drive_msg.drive.steering_angle_velocity = 0
        drive_msg.drive.speed = current_speed # subtract some epsilon?
        drive_msg.drive.acceleration = 0
        drive_msg.drive.jerk = 0
        self.safety_publisher.publish(drive_msg)


if __name__ == '__main__':
    rospy.init_node('safety_controller')
    safety_controller = SafetyController()
    rospy.spin()
