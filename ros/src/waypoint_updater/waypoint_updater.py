#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Bool

import numpy as np
from std_msgs.msg import Int32

import math, time

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

	self.waypoints = [] # List of waypoints
	self.final_waypoints = [] # List of the next waypoints for the car
        self.pose = None # vehicle pose
        self.traffic_id = None
	self.next_waypoint_idx = None
	self.timestamp_new = time.time()
	self.dbw_enabled = False
	self.reached_end = False


        ## Subscribers
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)  
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)  

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

	rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        ## Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement

        if not self.waypoints:
            rospy.logwarn("No base waypoints received")
            return

        self.pose = msg.pose

	if not self.dbw_enabled:
	    # Do a full search if dbw is disabled
	    self.next_waypoint_idx = None

        self._get_next_waypoints(self.pose.position)
	self._check_traffic_light()
	self._publish_waypoints()

    def _get_next_waypoints(self, car_position):
        min_dist = 0
        if not self.reached_end:
            if not self.next_waypoint_idx:
	        # rospy.logwarn("Doing a full search")
	        self.next_waypoint_idx, min_dist = self._search_waypoints(car_position, self.waypoints)
	    else:
	        # rospy.logwarn("Searching next few waypoints")
                waypoint_list = [self.waypoints[idx] for idx in range(self.next_waypoint_idx, self.next_waypoint_idx+10)]
                idx, min_dist = self._search_waypoints(car_position, waypoint_list)
	        self.next_waypoint_idx += idx

	#rospy.logwarn("current speed is %f", self.current_velocity.linear.x)
	#rospy.logwarn("current min_dist is %f", min_dist)

    def _check_traffic_light(self):

        #if abs(self.traffic_id - self.next_waypoint_idx) < 4:
	#    rospy.logwarn("Car is at the traffic light!")
        stopping_wp_dist = 125
        if self.traffic_id > 0: # A red light is near
            #rospy.logwarn("Traffic id is %d, and car is at %d", self.traffic_id, self.next_waypoint_idx)
	    if self.traffic_id < self.next_waypoint_idx + stopping_wp_dist:
		for idx in range(0,stopping_wp_dist):

                    dist = self.distance(self.waypoints, self.traffic_id-idx-5, self.traffic_id-5)

                    vel = dist*dist * 0.03 * 0.03

                    if vel > 10:
                        vel = 4
                    #vel = 0 + idx
	            if vel < 1 and dist < 15.:
                        vel = 0
                    else:
                        if vel < 1:
                            vel = 0.8

                    self.set_waypoint_velocity(self.waypoints, self.traffic_id-idx-4, vel)
        else:
            for idx in range(0,10):
                vel = 10
                self.set_waypoint_velocity(self.waypoints, self.next_waypoint_idx+idx, vel)

        if (self.next_waypoint_idx+LOOKAHEAD_WPS < len(self.waypoints)) and not self.reached_end:
	    final_waypoint = self.next_waypoint_idx+LOOKAHEAD_WPS
        else:
            self.reached_end = True
            final_waypoint = len(self.waypoints)

	#if self.next_waypoint_idx < self.

        self.final_waypoints = [self.waypoints[idx] for idx in range(self.next_waypoint_idx, final_waypoint)]

        if self.reached_end:
            for idx in range(0,150):
                dist = self.distance(self.waypoints, len(self.waypoints)-idx-5, len(self.waypoints)-5)

                vel = dist*dist * 0.01 * 0.01

                if vel > 10:
                   vel = 4

                if vel < 1:
                   vel = 0
	        #rospy.logwarn("%f", vel)

                self.set_waypoint_velocity(self.waypoints, len(self.waypoints)-idx-1, vel)
           
    def _publish_waypoints(self):
        waypoint_msg = Lane()
        waypoint_msg.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(waypoint_msg)

    def _search_waypoints(self, car_position, waypoint_list):
        min_dist = float("inf")
        min_dist_idx = 0

        dist = 0.0

        for idx in range(len(waypoint_list)):
	    waypoint_x = waypoint_list[idx].pose.pose.position.x
	    waypoint_y = waypoint_list[idx].pose.pose.position.y

            dist_x = car_position.x - waypoint_x
	    dist_y = car_position.y - waypoint_y
            dist = ((dist_x)*(dist_x) + (dist_y)*(dist_y))

            if dist < min_dist:
                min_dist = dist
                min_dist_idx = idx
        return min_dist_idx, dist

    def _local_search_waypoints(self, car_position):
        min_dist = float("inf")
        min_dist_idx = 0

        dist = 0.0

        return min_dist_idx


    def waypoints_cb(self, msg):
        # TODO: Implement
        # load all the of waypoints
        if not self.waypoints:
            self.waypoints = msg.waypoints
            rospy.logwarn("Not a warning, received %d waypoints", len(msg.waypoints))
	else:
            rospy.logwarn("No waypoints!")

    def velocity_cb(self, msg):
        # TODO: Callback for /current_velocity message. Implement
        self.current_velocity = msg.twist
	self.timestamp_old = self.timestamp_new
	self.timestamp_new = time.time()
	self.delta_t = self.timestamp_new - self.timestamp_old     

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_id = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
