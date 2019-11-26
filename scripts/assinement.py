#!/usr/bin/env python


# Python libs
import sys, time

#import numpy
import numpy

# OpenCV
import cv2

# Ros libraries
import roslib, rospy, image_geometry, tf, rospy, cv_bridge

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


   
class Robot_main:
    def __init__(self):
        self.bridge = CvBridge()

        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)

        rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",
            Image, self.image_callback)

        self.tf_listener = tf.TransformListener()

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once 
       
  
      
    def image_callback(self, data):
        cv2.namedWindow("window", 1)
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
         
             
        #all colours not cuttently useing 
        #lower_colour = numpy.array([0, 0, 0])
       #upper_colour = numpy.array([255, 255, 255])
       #mask = cv2.inRange(hsv, lower_colour, upper_colour)        
        #M = cv2.moments(mask)
     
       
        lower_green = numpy.array([0, 150, 0]) #bgr arrrays 
        upper_green = numpy.array([255, 255, 255])
        #lower_green = numpy.array([60, 200, 0]) #bgr arrrays 
        #upper_green = numpy.array([60, 255, 255])
        mask = cv2.inRange(hsv_img, lower_green, upper_green)        
        mGre = cv2.moments(mask)

        cv_image = cv2.resize(cv_image, (0,0), fx=0.5, fy=0.5)
        mask = cv2.resize(mask, (0,0), fx=0.5, fy=0.5)

        cv2.imshow("hsv", mask)
        cv2.imshow("window", cv_image)
        print 'running'
        cv2.waitKey(3)



        
cv2.startWindowThread()
rospy.init_node('robot_main')
robot_main = Robot_main()
rospy.spin()

cv2.destroyAllWindows()