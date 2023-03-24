#!/usr/bin/env python2

import numpy as np

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

class Drive:
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic", "/drive")
    #DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic", "/vesc/ackermann_cmd_mux/input/navigation")
    VELOCITY = rospy.get_param("wall_follower/velocity", 1)

    def __init__(self):
        self.carpub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

    def forward_drive(self):
        car = AckermannDriveStamped()
        car.header.stamp = rospy.Time.now()
        car.header.frame_id = "base_link"
        car.drive.speed = self.VELOCITY
        car.drive.steering_angle = 0
        self.carpub.publish(car)

    def stop_drive(self):
        car = AckermannDriveStamped()
        car.header.stamp = rospy.Time.now()
        car.header.frame_id = "base_link"
        car.drive.speed = 0
        car.drive.steering_angle = 0
        self.carpub.publish(car)
    



if __name__ == "__main__":
    rospy.init_node('drive_straight')
    drive_straight = Drive()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        drive_straight.forward_drive()
        rate.sleep()
    drive_straight.stop_drive()
    rospy.spin()
