#!/usr/bin/env python
import rospy
import numpy as np

# TODO: import ROS msg types and libraries
# import message_filters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive


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
            "/vesc/odom", Odometry, self.odom_callback)

        self.drive_topic = rospy.get_param("/nav_drive_topic")
        self.drive_pub = rospy.Publisher(
            self.drive_topic, AckermannDriveStamped, queue_size=10)
        self.brake_bool_pub = rospy.Publisher(
            "/brake_bool", Bool, queue_size=10)
        self.ttc_threshhold = 0.08

        # self.drive_topic = rospy.get_param('/vesc/high_level/ackermann_cmd_mux/input/auto_drive')
        # self.drive = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        # self.drive_msg = AckermannDriveStamped()
        # self.drive_msg.drive.speed = 1.0
        # self.drive.publish(drive_msg)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        # calculate TTC
        stationary = 0.001
        # print(self.speed)
        if abs(self.speed) > stationary:

            # print("scan_msg: ", scan_msg)

            steeringAngle = 0.00

            self.drive_st_msg = AckermannDriveStamped()
            self.drive_msg = AckermannDrive()

            if(self.speed > 1):
                self.speed = 0.5

            if(self.speed < 0):
                self.speed = 0

            if(self.speed <= 1.0):
                steeringAngle = steeringAngle / 0.75 * 0.9

            self.drive_msg.steering_angle = steeringAngle*-0.75
            self.speed = self.speed+0.75
            self.drive_msg.speed = self.speed

            print("steering angle: ", self.drive_msg.steering_angle)
            print("speed: ", self.drive_msg.speed)

            print("angle min: ",  scan_msg.angle_min)
            print("angle max: ",  scan_msg.angle_max)

            fixed_angle_min = scan_msg.angle_min + 1.57
            fixed_angle_max = scan_msg.angle_max - 1.57

            print("fixed angle min: ",  fixed_angle_min)
            print("fixed angle max: ",  fixed_angle_max)

            self.angles_array = np.arange(
                fixed_angle_min, fixed_angle_max, scan_msg.angle_increment)
            self.ranges_array = np.array(scan_msg.ranges)

            # fix denominator

            # option 1 --------
            # self.range_rates = np.max(
            #    self.speed * np.cos(self.angles_array), 0) + 0.000000001
            # self.ttcs = (self.ranges_array/self.range_rates)
            # --------------

            # option 2 ----------
            denominator = np.max(self.speed * np.cos(self.angles_array), 0)
            if (denominator == 0):
                self.ttcs = np.inf
            else:
                self.range_rates = denominator
                self.ttcs = (self.ranges_array/self.range_rates)
                # print("ttcs: ", self.ttcs)

            # ------------

            # find the minimum ttc value
            self.min_ttc = np.min(self.ttcs)
            print("min ttc: ", self.min_ttc)
            # brake_bool = Bool()

            # TODO: publish brake message and publish controller bool
            if self.min_ttc < self.ttc_threshhold:
                print("Min TTC below Threshhold, Apply brake here")
                # brake_bool.data = True
                self.drive_msg.speed = 0.0
                # self.brake_bool_pub.publish(brake_bool)
                self.brake_bool_pub.publish(True)

            else:
                # brake_bool.data = False
                # self.brake_bool_pub.publish(brake_bool)
                self.brake_bool_pub.publish(False)

            self.drive_st_msg.drive = self.drive_msg
            self.drive_pub.publish(self.drive_st_msg)


def main():
    rospy.init_node('yuntao_safety', anonymous=True)
    sn = Safety()
    rospy.spin()


if __name__ == '__main__':
    main()
