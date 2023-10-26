#!/usr/bin/env python3

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time

class Swift():
    def __init__(self):
        rospy.init_node('drone_control')
        self.drone_position = [0.0, 0.0, 0.0]
        self.setpoints = [
            [0, 0, 23],
            [2, 0, 23],
            [2, 2, 23],
            [2, 2, 25],
            [-5, 2, 25],
            [-5, -3, 25],
            [-5, -3, 21],
            [7, -3, 21],
            [7, 0, 21],
            [0, 0, 19]
        ]
	self.cmd = swift_msgs()
	self.cmd.rcRoll = 1000
	self.cmd.rcPitch = 1500
	self.cmd.rcYaw = 1000
	self.cmd.rcThrottle = 1500
	self.cmd.rcAUX1 = 1500
	self.cmd.rcAUX2 = 1500
	self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500
        self.setpoint_index = 0
        self.error_threshold = 0.2
        self.Kp = [37.17, 20.9, 90.2]
        self.Ki = [0.008, 0.0008, 0.0008]
        self.Kd = [1002.4, 911.7, 1323.2]
        self.max_throttle = 2000
        self.min_throttle = 1000

        self.error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.sum_all_error = [0.0, 0.0, 0.0]
		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)

		#------------------------Add other ROS Publishers here-----------------------------------------------------

		self.alt_error_pub = rospy.Publisher('/alt_error' , Float64 , queue_size=1)
		self.req_error_pub = rospy.Publisher('/req_error', Float64 , queue_size=1)
		self.upper_req_error_pub = rospy.Publisher('/upper_req_error', Float64 , queue_size=1)
		self.lower_req_error_pub = rospy.Publisher('/lower_req_error', Float64 , queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error' , Float64 , queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error' , Float64 , queue_size=1)
# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		
		#-------------------------Add other ROS Subscribers here----------------------------------------------------

		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch', PidTune , self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll', PidTune , self.roll_set_pid)



		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        self.pid_controller()

    def pid_controller(self):
        self.error[0] = self.setpoints[self.setpoint_index][0] - self.drone_position[0]  # Error for x axis
        self.error[1] = self.setpoints[self.setpoint_index][1] - self.drone_position[1]  # Error for y axis
        self.error[2] = self.setpoints[self.setpoint_index][2] - self.drone_position[2]  # Error for z axis

        
        for i in range(3):
            self.cmd.rcThrottle = int(1500 + self.error[i] * self.Kp[i] +
                                 (self.error[i] - self.prev_error[i]) * self.Kd[i] +
                                 self.sum_all_error[i] * self.Ki[i])

            # Limiting throttle values
            if self.cmd.rcThrottle > self.max_throttle:
                self.cmd.rcThrottle = self.max_throttle
            elif self.cmd.rcThrottle < self.min_throttle:
                self.cmd.rcThrottle = self.min_throttle

            self.prev_error[i] = self.error[i]
            self.sum_all_error[i] += self.error[i]

            if i == 0:
                self.cmd.rcPitch = cmd.rcThrottle
            elif i == 1:
                self.cmd.rcRoll = cmd.rcThrottle

            self.command_pub.publish(cmd)

        # Check if the drone is within the error threshold of the current waypoint
        if all(abs(err) <= self.error_threshold for err in self.error):
            self.setpoint_index += 1

            # Check if all waypoints have been reached
            if self.setpoint_index >= len(self.setpoints):
                rospy.loginfo("All waypoints reached!")
                rospy.signal_shutdown("Waypoints reached.")
            else:
                rospy.loginfo("Moving to next waypoint: {}".format(self.setpoints[self.setpoint_index]))

            # Reset error sums for the next waypoint
            self.sum_all_error = [0.0, 0.0, 0.0]

        # Publish error values for monitoring
        self.alt_error_pub.publish(self.error[2])
        self.pitch_error_pub.publish(self.error[0])
        self.roll_error_pub.publish(self.error[1])

if __name__ == '__main__':
    try:
        controller = SwiftController()
        rospy.Subscriber('whycon/poses', PoseArray, controller.whycon_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
s
