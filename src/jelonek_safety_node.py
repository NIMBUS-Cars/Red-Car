# #!/usr/bin/env python


# import rospy
# import sys
# import math
# from std_msgs.msg import Bool
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from ackermann_msgs.msg import AckermannDriveStamped


# # Karissa Jelonek
# # CSE 860
# # Lab 2

# class Safety(object):

#     def __init__(self):
#         self.speed = 0
#         rospy.Subscriber('scan', LaserScan, self.scan_callback)
#         rospy.Subscriber('odom', Odometry, self.odom_callback)
#         self.ackermannDrivePub = rospy.Publisher('vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=1)
#         self.boolPub = rospy.Publisher('brake_bool', Bool, queue_size=1)

#     def odom_callback(self, odom_msg):
#         self.speed = odom_msg.twist.twist.linear.x

#     def scan_callback(self, scan_msg):
#         r = []
#         min_ttc = 1000000
#         r = scan_msg.ranges
#         i = 0
#         for i in range(len(r)):
#             if r[i] > scan_msg.range_min and r[i] < scan_msg.range_max:
#                 r_dot = self.speed * math.cos(scan_msg.angle_min + scan_msg.angle_increment * i)
#                 r_dot = max(r_dot, 0)
#                 if r_dot != 0 and r[i] / r_dot < min_ttc:
#                     min_ttc = r[i] / r_dot
#         #print("min_ttc ", min_ttc)
#         if min_ttc < 0.7:
#             msg = AckermannDriveStamped()
#             msg.drive.speed = 0.0
#             self.ackermannDrivePub.publish(msg)

#             brake_bool = Bool()
#             brake_bool = True
#             self.boolPub.publish(brake_bool)
#             print("stop ", min_ttc )


# def main():
#     rospy.init_node('safety_node')
#     sn = Safety()
#     rospy.spin()

# if __name__ == '__main__':
#     main()
