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
        self.speed = 0.3
        # TODO: create ROS subscribers and publishers.
        self.laser_scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        # self.odom_sub = rospy.Subscriber("/vesc/odom", Odometry, self.odom_callback)
        self.drive_topic = rospy.get_param("/nav_drive_topic")
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=10)
        self.brake_bool_pub = rospy.Publisher("/brake_bool", Bool, queue_size=10)
        self.ttc_threshhold = 0.5

    # def odom_callback(self, odom_msg):
    #     # TODO: update current speed
    #     self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        # calculate TTC
        stationary = 0.001
        # print(self.speed)
        if abs(self.speed) > stationary:
            # print("scan_msg: ", scan_msg)
            self.drive_st_msg = AckermannDriveStamped()
            self.drive_msg = AckermannDrive()
            rospy.loginfo("speed: ", self.drive_msg.speed)

            # steeringAngle = 0.00
            # if(self.speed <= 0.2):
            #     steeringAngle = steeringAngle / 0.75 * 0.9
            # rospy.loginfo("steering angle: ", self.drive_msg.steering_angle)

            #fixed_angle_min = scan_msg.angle_min + 1.57
            #fixed_angle_max = scan_msg.angle_max - 1.57
            fixed_angle_min = scan_msg.angle_min + 2
            fixed_angle_max = scan_msg.angle_max - 2
            rospy.loginfo("fixed angle min: ",  fixed_angle_min)
            rospy.loginfo("fixed angle max: ",  fixed_angle_max)

            self.angles_array = np.arange(fixed_angle_min, fixed_angle_max, scan_msg.angle_increment)
            self.ranges_array = np.array(scan_msg.ranges)
            rospy.loginfo("ranges_array: ",  self.ranges_array)

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
            # ------------
            # find the minimum ttc value
            self.min_ttc = np.min(self.ttcs)
            rospy.loginfo("min ttc: ", self.min_ttc)

            # TODO: publish brake message and publish controller bool
            if self.min_ttc < self.ttc_threshhold:
                rospy.loginfo("Min TTC below Threshhold, Apply brake here")
                self.brake_bool_pub.publish(True)
                self.speed = 0.0

            else:
                self.brake_bool_pub.publish(False)

            # self.drive_msg.steering_angle = steeringAngle*-0.75
            self.drive_msg.speed = self.speed
            self.drive_st_msg.drive = self.drive_msg
            self.drive_pub.publish(self.drive_st_msg)


def main():
    rospy.init_node('yuntao_safety', anonymous=True)
    sn = Safety()
    rospy.spin()


if __name__ == '__main__':
    main()
