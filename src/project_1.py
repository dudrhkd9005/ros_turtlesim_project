#!/usr/bin/python
#-*- encoding: utf-8 -*-

import cv2, rospy, time
import numpy as np
from math import pow, atan2, sqrt

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from turtlesim.msg import Pose

bridge = CvBridge()
img = np.empty(shape=[0])
x, y, theta = 0, 0, 0
isCheck = False

control_msg = Twist()

class GoTurtle:
    def __init__(self):
        rospy.Subscriber("/turtle1/pose", Pose, self.update_pose)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def move_distance(self, pose):
        return sqrt(pow((pose.x - self.pose.x), 2) + 
                    pow((pose.y - self.pose.y), 2))

    def linear_vel(self, pose, constant=1.5):
        return constant * self.move_distance(pose)

    def steering_angle(self, pose):
        return atan2(pose.y - self.pose.y, pose.x - self.pose.x)

    def angular_vel(self, pose, constant=6):
        return constant * (self.steering_angle(pose) - self.pose.theta)

    def control_msg_publish(self, pose):
        control_msg.linear.x = self.linear_vel(pose)
        control_msg.linear.y = 0
        control_msg.linear.z = 0
        control_msg.angular.x = 0
        control_msg.angular.y = 0
        control_msg.angular.z = self.angular_vel(pose)

        self.pub.publish(control_msg)

    def move_turtle(self, x, y, d):
        global isCheck
        pose = Pose()

        pose.x = x
        pose.y = y

        distance = d
        if(isCheck):
            while self.move_distance(pose) >= distance:
                self.control_msg_publish(pose)
                
            isCheck = False

        control_msg.linear.x = 0
        control_msg.angular.z = 0
        self.pub.publish(control_msg)
        #rospy.spin()


def image_callback(img_data):
    global bridge
    global img
    img = bridge.imgmsg_to_cv2(img_data, "bgr8")

if __name__ == "__main__":
    rospy.init_node("foscar_project")
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    time.sleep(1)
    while not rospy.is_shutdown():

        # 이미지 bgr -> hsv 변환해서 저장하기
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([90,50,70])
        upper = np.array([128,255,255])
        img_mask = cv2.inRange(img_hsv, lower, upper)
        img_result = cv2.bitwise_and(img, img, mask=img_mask)

        roi = [img_result[0:160, 0:213], img_result[0:160, 213:426], img_result[0:160, 426:640]
             , img_result[160:320, 0:213], img_result[160:320, 213:426], img_result[160:320, 426:640]
             , img_result[320:480, 0:213], img_result[320:480, 213:426], img_result[320:480, 426:640]]

        # 격자무늬 시각화
        cv2.line(img, (0, 160), (639, 160), (0, 0, 0), 1)
        cv2.line(img, (0, 320), (639, 320), (0, 0, 0), 1)
        cv2.line(img, (210, 0), (210, 480), (0, 0, 0), 1)
        cv2.line(img, (420, 0), (420, 480), (0, 0, 0), 1)

        # 비디오 화면 띄우기
        # 동시에 여러 창 띄우는것도 가능
        cv2.imshow("image", img)
        #cv2.imshow("hsv_image", img_hsv)
        cv2.imshow("result_image", img_result)

        #for i in range(9):

        cv2.imshow("roi", roi[0])
        b = img_result[50,50][0]
        g = img_result[50,50][1]
        r = img_result[50,50][2]
        #print(img_result[50,50][0])

        detect_threshold = 50000
        turtle = GoTurtle()

        nonzero = [0 for i in range(9)]
        nonzeroy = [0 for i in range(9)]
        nonzerox = [0 for i in range(9)]

        pose_x = [2, 6, 10, 2, 6, 10, 2, 6, 10]
        pose_y = [10, 10, 10, 6, 6, 6, 2, 2, 2]
        distance = 0.5

        #거북이제어
        for i in range(9):
            nonzero[i] = roi[i].nonzero()
            nonzeroy[i] = np.array(nonzero[i][0])
            nonzerox[i] = np.array(nonzero[i][1])
            #print(str(i+1)+":", len(nonzerox[i]))
            if(len(nonzerox[i]) > detect_threshold):
                isCheck = True
                turtle.move_turtle(pose_x[i], pose_y[i], distance)
                time.sleep(1)


        if cv2.waitKey(1) & 0xff == ord("q"):
            break

    cv2.destroyAllWindows()
