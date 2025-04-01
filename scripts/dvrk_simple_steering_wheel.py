#!/usr/bin/env python

import dvrk
import math
import sys
import rospy
import numpy
#import PyKDL
import threading
from sensor_msgs.msg import Joy
from cisst_msgs.msg import prmCartesianImpedanceGains
from std_msgs.msg import Float32

# Example class for a simple steering wheel using the dVRK MTM arm
class dvrk_simple_steering_wheel:

    def configure(self, robot_name):
        """
        Configure the dVRK MTM arm and initialize necessary ROS subscribers and publishers.
        :param robot_name: Name of the MTM arm (MTML or MTMR)
        """
        self.arm = dvrk.mtm(robot_name)
        self.coag_event = threading.Event()
        
        # Subscribe to foot pedal coagulation event
        rospy.Subscriber('/dvrk/footpedals/coag',
                         Joy, self.coag_event_cb)
        
        # Publisher for wheel control messages
        self.wheel_pub = rospy.Publisher('/dvrk_carla/control/wheel',
                                         Float32, latch=True, queue_size=1)

    def home(self, arm):
        """
        Home the arm and move it to the zero position.
        :param arm: The dVRK MTM arm instance
        """
        print rospy.get_caller_id(), ' -> starting home'
        arm.home()
        
        # Get current joint positions to set the goal size
        goal = numpy.copy(arm.get_current_joint_position())
        
        # Set all joint positions to zero
        goal.fill(0)
        
        # Move arm to the zero position with interpolation
        arm.move_joint(goal, interpolate=True)

    def coag_event_cb(self, data):
        """
        Callback function for the coagulation pedal event.
        :param data: Joy message containing button states
        """
        if data.buttons[0] == 1:
            self.coag_event.set()

    def wait_for_coag(self):
        """
        Wait for the coagulation pedal to be pressed, with a timeout of 600 seconds.
        """
        self.coag_event.clear()
        self.coag_event.wait(600)

    def do_wheel_things(self):
        """
        Function to control the steering wheel based on the MTM joint position.
        """
        print rospy.get_caller_id(), ' -> press COAG pedal to set straight position'
        
        # Wait for the COAG pedal press to set the initial position
        self.wait_for_coag()
        
        # Reset force applied by the MTM arm
        self.arm.set_wrench_body_force((0.0, 0.0, 0.0))
        
        # Conversion factor from radians to wheel angle
        wheel_multiplier = 1.0 / 180.0
        
        rate = rospy.Rate(100)  # 100 Hz loop rate
        while not rospy.is_shutdown():
            
            # Create a wheel control message
            wheel_msg = Float32()
            
            # Compute wheel angle from joint position (index 5 is assumed to control the wheel rotation)
            wheel_msg.data = numpy.rad2deg(self.arm.get_current_joint_position()[5]) * wheel_multiplier
            
            # Publish the wheel control message
            self.wheel_pub.publish(wheel_msg)
            
            # Sleep to maintain loop rate
            rate.sleep()

    # Main method
    def run(self):
        """
        Execute the homing sequence and run the wheel control logic.
        """
        self.home(self.arm)
        self.do_wheel_things()

if __name__ == '__main__':
    try:
        # Ensure the correct number of arguments are provided
        if len(sys.argv) != 2:
            print sys.argv[0], ' requires argument, i.e. MTML or MTMR'
        else:
            # Create application instance and configure it with the provided MTM arm name
            application = dvrk_simple_steering_wheel()
            application.configure(sys.argv[1])
            application.run()

    except rospy.ROSInterruptException:
        pass
