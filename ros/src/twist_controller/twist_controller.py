from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

import rospy
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
	self.throttle_kp = 0.7
	self.throttle_ki = 0.15
	self.throttle_kd = 0.25

	self.steering_kp = 0.25
	self.steering_ki = 0.
	self.steering_kd = 0.2

	self.min_speed = 0.05
	
        self.throttle_controller = PID(self.throttle_kp, self.throttle_ki, self.throttle_kd)
	self.steering_controller = PID(self.steering_kp, self.steering_ki, self.steering_kd)

	self.steering_filter = LowPassFilter(0.75,0.2)

        self.yaw_control = YawController(kwargs['wheel_base'], kwargs['steer_ratio'], self.min_speed , kwargs['max_lat_accel'], kwargs['max_steer_angle'])
	
	self.last_sample_time = None
        #self.max_velocity = rospy.get_param('~velocity')
	self.max_velocity = rospy.get_param('/waypoint_loader/velocity') / 3.6 

    def control(self, proposed_linear_vel, proposed_angular_vel, current_linear_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
	if not dbw_enabled:
	    self.last_sample_time = rospy.get_time()
            self.throttle_controller.reset()
	    return 0., 0., 0.

	sample_time = rospy.get_time() - self.last_sample_time

	if proposed_linear_vel > self.max_velocity:
            proposed_linear_vel  = self.max_velocity

	error = proposed_linear_vel - current_linear_vel

	if error < 0:
            brake = 1.25*self.max_velocity*abs(error)
 	    #brake = max(brake, 1.0)
            throttle = 0.
	else:
	    brake = 0.
            throttle = self.throttle_controller.step(error, sample_time)
	
        steering = self.yaw_control.get_steering(proposed_linear_vel, proposed_angular_vel, current_linear_vel)
        steering = self.steering_filter.filt(steering)

        return throttle, brake, steering
