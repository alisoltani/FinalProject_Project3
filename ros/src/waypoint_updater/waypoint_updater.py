#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import numpy as np
from std_msgs.msg import Int32

import math

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


        ## Subscribers
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)  


        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        ## Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement

        if not self.waypoints:
            rospy.logwarn("No base waypoints received")
            return

        self.pose = msg.pose

	car_position = self.pose.position

        self._get_next_waypoints(car_position)
	self._publish_waypoints()

    def _get_next_waypoints(self, car_position):
	search_for_global_minima = False

        if not self.next_waypoint_idx:
	    search_for_global_minima = True
	

	search_for_global_minima = True

	if search_for_global_minima:
	    self.next_waypoint_idx = self._full_search_waypoints(car_position)
	else:
            return

        self.final_waypoints = [self.waypoints[idx] for idx in range(self.next_waypoint_idx, self.next_waypoint_idx+LOOKAHEAD_WPS)]

    def _publish_waypoints(self):
        waypoint_msg = Lane()
        waypoint_msg.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(waypoint_msg)

    def _full_search_waypoints(self, car_position):
        min_dist = float("inf")
        min_dist_idx = 0

        dist = 0.0

        for idx in range(len(self.waypoints)):
	    waypoint_x = self.waypoints[idx].pose.pose.position.x
	    waypoint_y = self.waypoints[idx].pose.pose.position.y

            dist_x = car_position.x - waypoint_x
	    dist_y = car_position.y - waypoint_y
            dist = ((dist_x)*(dist_x) + (dist_y)*(dist_y))

            if dist < min_dist:
                min_dist = dist
                min_dist_idx = idx

        return min_dist_idx


    def waypoints_cb(self, msg):
        # TODO: Implement
        # load all the of waypoints
        if not self.waypoints:
            self.waypoints = msg.waypoints
            rospy.logwarn("Not a warning, received %d waypoints", len(msg.waypoints))
	else:
            rospy.logwarn("No waypoints!")

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_id = msg.data
        pass

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
