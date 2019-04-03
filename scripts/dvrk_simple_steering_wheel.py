#!/usr/bin/env python

import dvrk
import math
import sys
import rospy
import numpy
import PyKDL
import threading
from sensor_msgs.msg import Joy
from cisst_msgs.msg import prmCartesianImpedanceGains
from std_msgs.msg import Float32

# example of application using arm.py
class dvrk_simple_steering_wheel:

    def configure(self, robot_name):
        self.arm = dvrk.mtm(robot_name)
        self.coag_event = threading.Event()
        rospy.Subscriber('/dvrk/footpedals/coag',
                         Joy, self.coag_event_cb)

        self.wheel_pub = rospy.Publisher('/dvrk_carla/control/wheel',
                                             Float32, latch = True, queue_size = 1)


    def home(self, arm):
        print rospy.get_caller_id(), ' -> starting home'
        arm.home()
        # get current joints just to set size
        goal = numpy.copy(arm.get_current_joint_position())
        # go to zero position
        goal.fill(0)
        arm.move_joint(goal, interpolate = True)

    def coag_event_cb(self, data):
        if (data.buttons[0] == 1):
            self.coag_event.set()

    def wait_for_coag(self):
        self.coag_event.clear()
        self.coag_event.wait(600)


    def do_wheel_things(self):

        print rospy.get_caller_id(), ' -> press COAG pedal to set straight position'
        self.wait_for_coag()

        self.arm.set_wrench_body_force((0.0, 0.0, 0.0))

        wheel_multiplier = 1.0 / 180.0

        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():





          wheel_msg = Float32()
          wheel_msg.data = numpy.rad2deg(self.arm.get_current_joint_position()[5]) * wheel_multiplier
          self.wheel_pub.publish(wheel_msg)


          rate.sleep()






    # main method
    def run(self):
        self.home(self.arm)
        self.do_wheel_things()

if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            print sys.argv[0], ' requires argument, i.e. MTML or MTMR'
        else:
            application = dvrk_simple_steering_wheel()
            application.configure(sys.argv[1])
            application.run()

    except rospy.ROSInterruptException:
        pass
