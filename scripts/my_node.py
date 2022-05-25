#!/usr/bin/env python

import rospy, message_filters
import imutils
import cv2
import numpy as np
import transformations as trans
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from cv_bridge import CvBridge, CvBridgeError

class line_follower:
    def __init__(self, name):
        self.name = name

        self.ranges = [
            {'id': 0, 'lower': (0, 241, 92), 'upper': (8, 255, 255), 'name': 'red', 'RGBA': (1,0,0,1)},
            {'id': 1, 'lower': (27, 241, 92), 'upper': (33, 255, 255), 'name': 'yellow', 'RGBA': (0.5,0.5,0,1)},
            {'id': 2, 'lower': (115, 220, 92), 'upper': (130, 255, 255), 'name': 'blue', 'RGBA': (0,0,1,1)}]

        self.stopped = False
        self.timeStopped = None

        # Object for storing path
        self.path = Path()
        self.path.header.frame_id = "odom"

        # Subs and pubs
        self.subscriber_map = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.subscriber_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.publisher_map = rospy.Publisher('/ecte477/map', OccupancyGrid, queue_size=1)
        self.publisher_path = rospy.Publisher('/ecte477/path', Path, queue_size=1)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        colour_image_topic = "/camera/rgb/image_raw"
        depth_image_topic = "/camera/depth/image_raw"
        colour_sub = message_filters.Subscriber(colour_image_topic, Image)
        depth_sub = message_filters.Subscriber(depth_image_topic, Image)
        self.camera_sync = message_filters.TimeSynchronizer([colour_sub, depth_sub], 10)
        self.camera_sync.registerCallback(self.callback_camera)

        self.bridge = CvBridge()
        self.image = None

        # wait for other nodes to start
        while not rospy.is_shutdown():
            if self.image is not None:
                cv2.imshow('Window', cv2.resize(self.image, (1920/2, 1080/2)))
                cv2.waitKey(30)
        

    def callback_camera(self, colour_msg, depth_msg):
        rospy.loginfo('[%s] callback_camera()', self.name)
        colour_image = self.bridge.imgmsg_to_cv2(colour_msg, desired_encoding='bgr8')
        overlay = colour_image
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        # self.image = colour_image

        # lower_yellow = (15, 200, 100)
        # upper_yellow = (35, 255, 255)
        # lower_red = (0, 241, 92)
        # upper_red = (8, 255, 255)

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        h, w, d = colour_image.shape

        cxs = [None, None, None]
        cys = [None, None, None]

        for range in self.ranges:
            hsv = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, range['lower'], range['upper'])
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # mask out the frame, leave 20 px gap at 3/4 of the height (from top)
            if range['id'] != 2:
                mask[0 : 3*h/4, 0:w] = 0
                mask[3*h/4 + 20 : h, 0:w] = 0

            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                cv2.circle(overlay, (cx, cy), 20, (0,255,0), -1)

                # store the center coordinates in a arrays
                cxs[range['id']] = cx
                cys[range['id']] = cy
        
        # check for the blue contour if it is detected and if it is close enough
        if cxs[2] != None and 0 < depth_image[cys[2], cxs[2]] < 0.5 and not self.stopped:
            rospy.loginfo('Stopping!')
            self.stopped = True
            self.timeStopped = rospy.Time.now()
        
        # both lines are detected
        if cxs[0] != None and cxs[1] != None:
            err = (cxs[0]+cxs[1])/2 - w/2
            twist.linear.x = 0.15
            twist.angular.z = -float(err) / 1000
        # only yellow line is detected
        elif cxs[0] == None and cxs[1] != None:
            twist.linear.x = 0.15
            twist.angular.z = -0.15
        # only red line is detected
        elif cxs[0] != None and cxs[1] == None:
            twist.linear.x = 0.15
            twist.angular.z = 0.15
        # if none of the lines are detected => no movement

        # Stop if there is an obstacle on the way
        if 0 < depth_image[h/2, w/2] < 0.7 or self.stopped and rospy.Time.now() - self.timeStopped < rospy.Duration(secs=2):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        # else:
        #     self.stopped = False
        

        self.cmd_vel_pub.publish(twist)
        self.image = overlay

    # get the camera info
    # def callback_camera_info(self, camera_info):
    #     self.K = np.array(camera_info.K).reshape([3, 3])

    # Simply repeact the map data to the correct topic
    def callback_map(self, data):
        self.publisher_map.publish(data)
        
    # Turn the odometry info into a path and repeat it to the correct topic
    def callback_odom(self, data):
        pose = PoseStamped()
        pose.pose = data.pose.pose
        self.path.poses.append(pose)
        self.publisher_path.publish(self.path)



if __name__ == '__main__':
    rospy.sleep(3)
    rospy.init_node('line_follower', anonymous=True)
    rospy.loginfo('[line_follower] Starting Line Follower Module')
    lf = line_follower('line_follower')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('[line_follower] Shutting Down Line Follower Module')