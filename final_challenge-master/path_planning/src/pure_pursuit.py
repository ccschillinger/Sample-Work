#!/usr/bin/env python

from __future__ import division
import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        #Class variables:
        self.lookahead = 9.0
        self.speed = 10.0
        self.kp = 0.01
        self.goal_threshhold = 0.20
        self.turning_params = [10.0, 24.0, 0.05]
        self.open_throttle = [50.0, 35.0, 0.015]
        self.previous_index = None
        self.robot_segement_view = 2
        self.wheelbase_length = 3.0
        self.straightening_distance = 20.0
        self.braking_distance = 35.0
        self.drive_topic = "/tesse/drive"
        self.odom_topic = "/pf/pose/odom"

        #Subscribers and Publishers
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size = 1)
        self.segment_pub = rospy.Publisher("/closest_segment", Marker, queue_size=1)
        self.goal_visualizer = rospy.Publisher("/target", Marker, queue_size=1)

        #Car turns off at end
        rospy.on_shutdown(self.stop_car)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.make_np_array()
        self.trajectory.publish_viz(duration=0.0)
        
    def odometry_callback(self, odom):
        #If at finish, stop car
        if len(self.trajectory.points) == 0:
            self.stop_car()
            return
        pose = self.pose_2_coords(odom.pose.pose)
        x,y,theta = pose
        position = [x,y]

        #The indicies looking forward and backward form the previous closest index to deal with intersections. This is a fix from our two path method
        if self.previous_index is not None:
            look_forward_index, look_back_index = [min((self.previous_index +1) + self.robot_segement_view, len(self.trajectory.np_points) - 1), max(self.previous_index - self.robot_segement_view, 0)]
        else:
            look_forward_index, look_back_index = [min(len(self.trajectory.np_points) - 1, 4), 0]
        
        #Find the index of the closest segment, and find the distances to the start and end of that segement in order to determine the speed settings for the car
        index_closest_seg = np.argmin(np.array([self.segment_dist_squared(position,self.trajectory.np_points[i],self.trajectory.np_points[i+1]) for i in range(look_back_index, look_forward_index)])) + look_back_index
        square_progress = np.dot(position - self.trajectory.np_points[index_closest_seg], position - self.trajectory.np_points[index_closest_seg])
        square_braking_dist = np.dot(position - self.trajectory.np_points[index_closest_seg+1], position - self.trajectory.np_points[index_closest_seg+1])
        
        #Set the previous index to equal the current index for next round
        self.previous_index = index_closest_seg

        #find the goal point, consider not iterating on idex_closest_seg, but another variable?
        goal = None
        iter_index = index_closest_seg
        while goal is None:
            goal = self.line_circle_intersection(self.trajectory.np_points[iter_index],self.trajectory.np_points[iter_index+1],position,self.lookahead)
            iter_index += 1
            if iter_index >= len(self.trajectory.np_points)-1:
                break
        
        #if we do not find a goal point, set it equal to the end of the path
        if goal is None:
            goal = self.trajectory.np_points[-1]

        #visualize goal point
        self.publish_goal(self.goal_visualizer, goal)

        #set speedas an k values for turning or gunning it
        #if np.sqrt(square_progress) <= self.straightening_distance:
        #    self.speed, self.lookahead, self.kp = self.turning_params
        #elif np.sqrt(square_braking_dist) <= self.braking_distance:
        #    self.speed, self.lookahead, self.kp = self.turning_params
        #else:
        #    self.speed, self.lookahead, self.kp = self.open_throttle

        #if the distance to the goal is large, find the car frame vector and apply ackerman steering with P control
        if np.dot(position-goal, position-goal) >= self.goal_threshhold:
            map_frame_vector = goal - position
            car_frame_vector = np.array([
                map_frame_vector[0]*np.cos(-theta)-map_frame_vector[1]*np.sin(-theta),
                map_frame_vector[0]*np.sin(-theta)+map_frame_vector[1]*np.cos(-theta)])
            R = self.lookahead / (2.0*np.sin(np.arctan2(car_frame_vector[1], car_frame_vector[0])))
            delta = -np.arctan(self.wheelbase_length/R)*self.kp
            steering_angle = min(np.pi/6.05, max(-np.pi/6.05, delta))

            drive_cmd = AckermannDriveStamped()
            header = drive_cmd.header
            header.stamp = rospy.Time.now()
            header.frame_id = "map" 
            dcd = drive_cmd.drive
            dcd.speed = self.speed
            dcd.steering_angle = steering_angle
            self.drive_pub.publish(drive_cmd)
        else:
            self.stop_car()
            return
    #=========================== HELPERS ===========================

    
    def pose_2_coords(self, pose):
        ''' Turns a pose in x and y corrdinates
        '''
        return np.array([pose.position.x,pose.position.y, euler_from_quaternion(np.array([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]))[2]], dtype="float64")

    def segment_dist_squared(self, p, s1, s2):
        ''' Returns the sqaure of the distance to a segment
        '''
        lensquare = np.dot(s2-s1, s2-s1)
        if lensquare != 0:
            projection = s1 + max(0.0, min(1.0, np.dot(p-s1, s2-s1) / lensquare)) * (s2-s1)
            return np.dot(p-projection, p-projection)
        return np.dot(s1-p, s1-p) 

    def line_circle_intersection(self, point_1, point_2, center, radius):
        ''' Finds the intersection between a line and a circle, the basis of this method was taken from online
        '''
        a = np.dot(point_2-point_1, point_2-point_1) #a = v.dot(v)
        b = 2*(np.dot(point_2-point_1, point_1-center)) # b = 2*v.dot(point_1-q)
        c = np.dot(point_1, point_1) + np.dot(center, center) - 2*np.dot(point_1, center) - radius**2 
        discriminant = b**2 - 4*a*c
        if discriminant >= 0:
            sqrt_disc = np.sqrt(discriminant)
            a = (a if a > 0.0000001 else 0.0000001)
            if 0 <= max((-b + sqrt_disc)/(2.*a),(-b - sqrt_disc)/(2.*a)) <= 1:
                return point_1 + max((-b + sqrt_disc)/(2.*a),(-b - sqrt_disc)/(2.*a))*(point_2-point_1)
        return None

    def publish_goal(self, publish_topic, point):
        ''' shows our goal point
        '''
        marker = Marker()
        header = marker.header
        scale = marker.scale
        color = marker.color

        header.stamp = rospy.Time.now()
        header.frame_id = "/map"
        marker.type = Marker.SPHERE
        scale.x, scale.y, scale.z = [1.0, 1.0, 1.0]
        color.a, color.r = [1.0, 1.0]
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0
        publish_topic.publish(marker)

    def stop_car(self):
        ''' shuts off our car
        '''
        drive_cmd = AckermannDriveStamped()
        header = drive_cmd.header
        header.stamp = rospy.Time.now()
        header.frame_id = "map" 
        dcd = drive_cmd.drive
        dcd.speed = 0
        dcd.steering_angle = 0
        self.drive_pub.publish(drive_cmd)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
