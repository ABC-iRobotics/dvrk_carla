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

# Class for simulating a dVRK-based steering wheel control
class dvrk_steering_wheel:

    def configure(self, robot_name_l, robot_name_r):
        """Configures the left and right MTM arms and initializes ROS subscribers and publishers."""
        self.arm_l = dvrk.mtm(robot_name_l)
        self.arm_r = dvrk.mtm(robot_name_r)
        self.camera_event = threading.Event()
        self.abort = False
        rospy.Subscriber('/dvrk/footpedals/camera',
                         Joy, self.camera_event_cb)
        self.set_gains_r_pub = rospy.Publisher(self.arm_r._arm__full_ros_namespace + '/set_cartesian_impedance_gains',
                                             prmCartesianImpedanceGains, latch=True, queue_size=1)
        self.set_gains_l_pub = rospy.Publisher(self.arm_l._arm__full_ros_namespace + '/set_cartesian_impedance_gains',
                                             prmCartesianImpedanceGains, latch=True, queue_size=1)
        self.wheel_pub = rospy.Publisher('/dvrk_carla/control/wheel',
                                             Float32, latch=True, queue_size=1)

    def home(self, arm):
        """Homes the specified MTM arm to its zero position."""
        print rospy.get_caller_id(), ' -> starting home'
        arm.home()
        goal = numpy.copy(arm.get_current_joint_position())
        goal.fill(0)  # Set all joints to zero position
        arm.move_joint(goal, interpolate=True)

    def camera_event_cb(self, data):
        """Callback function for camera pedal event."""
        print rospy.get_caller_id(), ' -> c1'
        if data.buttons[0] == 1:
            print rospy.get_caller_id(), ' -> c2'
            self.abort = True
            self.camera_event.set()

    def wait_for_camera(self):
        """Waits for the camera pedal event to be triggered."""
        self.camera_event.clear()
        self.camera_event.wait(600)

    def quaternion_multiply(self, quaternion1, quaternion0):
        """Performs quaternion multiplication."""
        x0, y0, z0, w0  = quaternion0
        x1, y1, z1, w1 = quaternion1
        return numpy.array([x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                     -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0], dtype=numpy.float64)

    def do_wheel_things(self):
        """Controls the wheel motion using the MTM arms."""
        gains_l = prmCartesianImpedanceGains()
        gains_r = prmCartesianImpedanceGains()

        # Set orientation to identity quaternions
        gains_l.ForceOrientation.w = 1.0
        gains_l.TorqueOrientation.w = 1.0
        gains_r.ForceOrientation.w = 1.0
        gains_r.TorqueOrientation.w = 1.0
        self.arm_l.lock_orientation_as_is()
        self.arm_r.lock_orientation_as_is()

        print rospy.get_caller_id(), ' -> press CAMERA pedal to set straight position'
        self.wait_for_camera()

        # Calculate the center of rotation
        x_offset = 0.35
        currpos_l = self.arm_l.get_current_position()
        currpos_r = self.arm_r.get_current_position()
        center_r = numpy.array([(currpos_l.p[0] + currpos_r.p[0] - x_offset) / 2.0, 
                                (currpos_l.p[1] + currpos_r.p[1]) / 2.0, 
                                (currpos_l.p[2] + currpos_r.p[2]) / 2.0])
        radius = numpy.abs(center_r[0] - currpos_r.p[0])
        center_l = numpy.array([center_r[0] + x_offset, center_r[1], center_r[2]])

        wheel_multiplier = 1.0 / 180.0

        # Set force and damping gains
        gains_l.PosStiffNeg.y = gains_l.PosStiffPos.y = -400.0
        gains_l.PosDampingNeg.y = gains_l.PosDampingPos.y = -3.0
        gains_r.PosStiffNeg.y = gains_r.PosStiffPos.y = -400.0
        gains_r.PosDampingNeg.y = gains_r.PosDampingPos.y = -1.0

        rate = rospy.Rate(100)  # 100Hz loop rate
        self.abort = False
        while not (rospy.is_shutdown() or self.abort):
            currpos_l = self.arm_l.get_current_position()
            currpos_r = self.arm_r.get_current_position()
            steer_angle_l, target_l = self.get_steer_angle(radius, center_l, currpos_l)
            steer_angle_r, target_r = self.get_steer_angle(radius, center_r, currpos_r)

            gains_l.ForcePosition.x = target_l[0]
            gains_l.ForcePosition.z = target_l[2]
            gains_r.ForcePosition.x = target_r[0]
            gains_r.ForcePosition.z = target_r[2]

            self.set_gains_l_pub.publish(gains_l)
            self.set_gains_r_pub.publish(gains_r)

            wheel_msg = Float32()
            wheel_msg.data = numpy.rad2deg(steer_angle_l) * wheel_multiplier
            self.wheel_pub.publish(wheel_msg)

            rate.sleep()

        print rospy.get_caller_id(), ' -> Aborted by pressing the CAMERA pedal'
        self.arm_l.move(self.arm_l.get_desired_position())
        self.arm_r.move(self.arm_r.get_desired_position())

    def get_steer_angle(self, radius, center, currpos):
        """Calculates the steering angle and target position."""
        pos = numpy.array([currpos.p[0], currpos.p[1], currpos.p[2]])
        direction = pos - center
        direction = direction / numpy.linalg.norm(direction)
        target = center + (direction * radius)
        steer_angle = numpy.arctan2(direction[2], direction[0])
        return steer_angle, target

    def run(self):
        """Main execution method."""
        self.home(self.arm_l)
        self.home(self.arm_r)
        self.do_wheel_things()

if __name__ == '__main__':
    try:
        if len(sys.argv) != 3:
            print sys.argv[0], ' requires two arguments, i.e. MTML and MTMR'
        else:
            application = dvrk_steering_wheel()
            application.configure(sys.argv[1], sys.argv[2])
            application.run()
    except rospy.ROSInterruptException:
        pass
