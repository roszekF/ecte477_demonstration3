#!/usr/bin/env python

import rospy, message_filters
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class line_follower:
    def __init__(self, name):
        self.name = name

        colour_image_topic = "/camera/rgb/image_raw"
        depth_image_topic = "/camera/depth/image_raw"
        colour_sub = message_filters.Subscriber(colour_image_topic, Image)
        depth_sub = message_filters.Subscriber(depth_image_topic, Image)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.camera_sync = message_filters.TimeSynchronizer([colour_sub, depth_sub], 10)
        self.camera_sync.registerCallback(self.callback_camera)

        self.bridge = CvBridge()
        self.image = None
        while not rospy.is_shutdown():
            if self.image is not None:
                cv2.imshow('Window', cv2.resize(self.image, (1920/2, 1080/2)))
                cv2.waitKey(30)
        


    def callback_camera(self, colour_msg, depth_msg):
        rospy.loginfo('[%s] callback_camera()', self.name)
        colour_image = self.bridge.imgmsg_to_cv2(colour_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        # self.image = colour_image

        lower_yellow = (15, 200, 100)
        upper_yellow = (35, 255, 255)

        hsv = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)


        h, w, d = colour_image.shape
        # mask out the frame, leave 20 px gap at 3/4 of the height (from top)
        mask[0 : 3*h/4, 0:w] = 0
        mask[3*h/4 + 20 : h, 0:w] = 0

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            cv2.circle(colour_image, (cx, cy), 20, (255,0,0), -1)

            err = cx - w/2
            twist.linear.x = 0.2
            twist.angular.z = -float(err) / 1000


        self.image = colour_image

        self.cmd_vel_pub.publish(twist)



if __name__ == '__main__':
    rospy.init_node('line_follower', anonymous=True)
    rospy.loginfo('[line_follower] Starting Line Follower Module')
    lf = line_follower('line_follower')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('[line_follower] Shutting Down Line Follower Module')