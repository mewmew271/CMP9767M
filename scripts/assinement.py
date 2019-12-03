#!/usr/bin/env python
#sudo pip2 install imutils
import imutils
# Python libs
import sys, time

#import numpy
import numpy

# OpenCV
import cv2

# Ros libraries
import roslib, rospy, image_geometry, tf, rospy, cv_bridge
import actionlib

# Ros Messages
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist 
from cv_bridge import CvBridge, CvBridgeError

#coordsFound = [[8.6, 8], [-8, -8]]#the objects coords that have yet to be moved to 
coordsFound = []
coordsBeen = []#the objects coords that have been moved too 
objectCoords = [[8.9, 8]]#the objects coords to move too 
   
class Robot_main:
	def __init__(self):
		# What to do if shut down (e.g. Ctrl-C or failure)
		rospy.on_shutdown(self.shutdown)

		self.bridge = CvBridge()

		self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info', 
			CameraInfo, self.camera_info_callback)

		rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",
			Image, self.image_callback)

		self.tf_listener = tf.TransformListener()

		self.goal_sent = False

		# What to do if shut down (e.g. Ctrl-C or failure)
		rospy.on_shutdown(self.shutdown)
		
		# Tell the action client that we want to spin a thread by default
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait for the action server to come up")

		# Allow up to 5 seconds for the action server to come up
		self.move_base.wait_for_server(rospy.Duration(5))

	def camera_info_callback(self, data):
		self.camera_model = image_geometry.PinholeCameraModel()
		self.camera_model.fromCameraInfo(data)
		self.camera_info_sub.unregister() #Only subscribe once 
	   
  
	  
	def image_callback(self, data):
		cv2.namedWindow("window", 1)
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		#part of move 
		

	 #carrots (basil)
	 #0, 150, 0
	 #255, 255, 255
	 #carrots (weeds)
	 #40, 10, 30
	 #100, 130, 110

	   
		lower_green = numpy.array([44, 12, 30]) #bgr arrrays 
		upper_green = numpy.array([80, 100, 70])
		mask = cv2.inRange(hsv_img, lower_green, upper_green)        
		mGre = cv2.moments(mask)
		
		cv_image = cv2.resize(cv_image, (0,0), fx=0.5, fy=0.5)
		mask = cv2.resize(mask, (0,0), fx=0.5, fy=0.5)
		mask = cv2.fastNlMeansDenoising(mask, None, 7, 7, 5) 
		#blur to remove some noise 
		mask = cv2.medianBlur(mask, 13)
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)

		#print how many objects have been found 
		print("I found {} objects".format(len(cnts)))
		for c in cnts:
		# calculate moments of binary image
			M = cv2.moments(c) 			

		# calculate x,y coordinate of center of found object  
		# add objects coords to coordsStack array [x, y]
		#if set(objectCoords).isnotsubset(set(coordsStack)):

			#find centoid of object 
			if M["m10"] or M["m00"] or M["m01"] != 0:#error checking 
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				centoid = cX, cY
				print(" ")
				print(centoid)
				#objectCoords = centoid transformed 
				#check if coords have all ready been added 
				#print'coordsFound: ', coordsFound
				print'coordsBeen: ', coordsBeen
				print'objectCoords: ', objectCoords
				print(objectCoords not in coordsBeen and not len(objectCoords) == 0)
				print(len(coordsFound))
				if objectCoords not in coordsBeen and not len(objectCoords) == 0:
					#coordsFound.append(objectCoords)#add new coords to found coords
					#here 
					position = {'x':objectCoords[0][0], 'y' : objectCoords[0][1]}
					quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

					rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
					success = robot_main.goto(position, quaternion)

					if success:
						rospy.loginfo("Hooray, reached the desired pose")
						coordsBeen.append(objectCoords[0])
						objectCoords.pop(0)
						#print'coordsFound: ', coordsFound
						print'coordsBeen: ', coordsBeen

					else:
						rospy.loginfo("The base failed to reach the desired pose")


				# highlight the centoid
				cv2.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)
		cv2.imshow("hsv", mask)
		cv2.imshow("window", cv_image)

		#show images on screen 
		
		
		cv2.waitKey(3)


	def goto(self, pos, quat):

		# Send a goal
		self.goal_sent = True
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

		# Start moving
		self.move_base.send_goal(goal)

		# Allow up to 60 seconds to complete task
		success = self.move_base.wait_for_result(rospy.Duration(60)) 

		state = self.move_base.get_state()
		result = False

		if success and state == GoalStatus.SUCCEEDED:
				# We made it!
			result = True
		else:
			self.move_base.cancel_goal()

		self.goal_sent = False
		return result



	def shutdown(self):
		if self.goal_sent:
			self.move_base.cancel_goal()
		rospy.loginfo("Stop")
		rospy.sleep(1)
		
cv2.startWindowThread()
rospy.init_node('robot_main')
robot_main = Robot_main()
rospy.spin()

cv2.destroyAllWindows()
