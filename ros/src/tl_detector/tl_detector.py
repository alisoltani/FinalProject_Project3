#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight, Waypoint
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3
LOOK_AHEAD_DISTANCE = 100

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.light_state = 4

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if (state == TrafficLight.RED or state == TrafficLight.YELLOW) else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose, waypoints):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint

        """
	index = None
	min_dist = float("inf")

	for ind in range(len(waypoints)):
            waypoint = waypoints[ind].pose.pose.position
            dist = ( (pose.x - waypoint.x)*(pose.x - waypoint.x) + (pose.y - waypoint.y)*(pose.y - waypoint.y)) 

	    if dist < min_dist:
                min_dist = dist
                index = ind

        return index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False, 0.
	
        if callable(self.light_classifier):
            return False, 0.

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
	    # Find the car position from waypoint list
            car_position_idx = self.get_closest_waypoint(self.pose.pose.position, self.waypoints.waypoints)
	    # Find the nearest traffic light to car from traffic light list
            light_idx = self.get_closest_waypoint(self.waypoints.waypoints[car_position_idx].pose.pose.position, self.lights)
        #TODO find the closest visible traffic light (if one exists)
            light_waypoint_idx = self.get_closest_waypoint(self.lights[light_idx].pose.pose.position, self.waypoints.waypoints)


	    light_position = self.waypoints.waypoints[light_waypoint_idx].pose.pose.position
	    light = self.lights[light_idx]
	    
	    stop_light_line = self.config['stop_line_positions'][light_idx]
	    stop_line = Waypoint()
            stop_line.pose.pose.position.x = stop_light_line[0]
	    stop_line.pose.pose.position.y = stop_light_line[1]
	    stop_line.pose.pose.position.z = 0.

	    stop_line_index = self.get_closest_waypoint(stop_line.pose.pose.position, self.waypoints.waypoints)
            stop_line_waypoint = self.waypoints.waypoints[stop_line_index].pose.pose.position

        if light and (stop_line_index > car_position_idx):
            state, confidence = self.get_light_state(light)
	    state_ground_truth = self.lights[light_idx].state
	    if state != self.light_state:
                self.light_state = state
                #rospy.logwarn("light changed from %d to %d (with confidence: %f) and ground truth is %d", self.light_state, state, confidence, state_ground_truth)

            return stop_line_index, self.light_state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
