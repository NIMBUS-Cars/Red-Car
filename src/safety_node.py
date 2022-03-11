#!/usr/bin/env python
import rospy
import numpy as np

# TODO: import ROS msg types and libraries
# import message_filters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


class Safety(object):
    """
    The class that handles emergency braking.
    """

    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.0

        # TODO: create ROS subscribers and publishers.
        self.laser_scan_sub = rospy.Subscriber(
            "/scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber(
            "/odom", Odometry, self.odom_callback)

        self.brake_pub = rospy.Publisher(
            "/vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=10)
        self.brake_bool_pub = rospy.Publisher(
            "/brake_bool", Bool, queue_size=10)
        self.brake_msg = AckermannDriveStamped()
        self.ttc_threshhold = 0.35

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # print("scan msg", scan_msg)
        # TODO: calculate TTC
        # calculate TTC
        stationary = 0.001
        print("speed: ", self.speed)
        if abs(self.speed) > stationary:
            self.angles_array = np.arange(
                scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
            self.ranges_array = np.array(scan_msg.ranges)
            print("angles_array: ", self.angles_array)
            print("ranges_array: ", self.ranges_array)


            # fix denominator

            # option 1 --------
            # self.range_rates = np.max(
            #    self.speed * np.cos(self.angles_array), 0) + 0.000000001
            # self.ttcs = (self.ranges_array/self.range_rates)
            # --------------

            # option 2 ----------
            denominator = np.max(self.speed * np.cos(self.angles_array), 0)
            print("denominator: ", denominator)
            if (denominator == 0):
                self.ttcs = np.inf
            else:
                self.range_rates = denominator
                self.ttcs = (self.ranges_array/self.range_rates)
                print("ttcs: ", self.ttcs)

            # ------------

            # find the minimum ttc value
            self.min_ttc = np.min(self.ttcs)
            # brake_bool = Bool()
            print("Min TTC: ", self.min_ttc)

            # TODO: publish brake message and publish controller bool
            if self.min_ttc < self.ttc_threshhold:
                print("Min TTC below Threshhold, Apply brake here: ", self.min_ttc)
                # brake_bool.data = True
                self.brake_msg.drive.speed = 0.0
                # self.brake_bool_pub.publish(brake_bool)
                self.brake_bool_pub.publish(True)
                self.brake_pub.publish(self.brake_msg)

            else:
                # brake_bool.data = False
                # self.brake_bool_pub.publish(brake_bool)
                self.brake_bool_pub.publish(False)


def main():
    rospy.init_node('yuntao_safety', anonymous=True)
    sn = Safety()
    rospy.spin()


if __name__ == '__main__':
    main()
