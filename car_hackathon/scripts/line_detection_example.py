#!/usr/bin/env python
#coding=utf-8

import rospy
from car_msgs.msg import MotorsControl
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

MAX_PWM = 255
is_trackbars = False

def truncate(pwm):
    if pwm < -MAX_PWM:
        return -MAX_PWM
    if pwm > MAX_PWM:
        return MAX_PWM
    return pwm

# Управление моторами
def motors(l, r):
    msg = MotorsControl()
    msg.left = truncate(l)
    msg.right = truncate(r)
    pub.publish(msg)




def image_callback(msg):
    image = bridge.imgmsg_to_cv2(msg, 'bgr8')

    # image cropping
    image_cropped = image[150:]

    # HSV conversion and binarisation
    hsv = cv2.cvtColor(image_cropped, cv2.COLOR_BGR2HSV)
    binary = cv2.inRange(hsv, (0,100,100), (30,255,255))

    # Canny edges estimation
    edges = cv2.Canny(binary, 100, 200)

    m = cv2.moments(binary);

    if m['m00'] > 0:
        cx = int(m['m10'] / m['m00'])
        cy = int(m['m01'] / m['m00'])

        cv2.circle(binary, (cx, cy), 20, (255,0,0), -1)

        err = cx - len(binary) / 2

        motors(20, 20-float(err)/20)
            
    cv2.imshow('img', image)
    cv2.imshow('binary', binary)
    cv2.waitKey(1)


def nothing(x):
    pass


if __name__ == '__main__':
    rospy.init_node('line_mover')
    rospy.loginfo('1')

    bridge = CvBridge()
    
    pub = rospy.Publisher('/motors_commands', MotorsControl, queue_size = 10)
    sub = rospy.Subscriber('/car_gazebo/camera1/image_raw', Image, image_callback)

    rospy.spin()
