#!/usr/bin/env python
#sudo pip2 install imutils
import imutils
# Python libs
import sys, time, copy

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
from nav_msgs.msg import Odometry


objectsLocated = []
coordsBeen = [[-10000, -10000]]#the objects coords that have been moved too -10000 temp value that will not exist
objectCoords = [0, 0]#the objects coords to move too 

coordsTop = [[5.484, -3.784], [5.484, -2.785], [5.484, -0.761], [5.484, 0.233], [5.484, 2.188], [5.484, 3.231]]
coordsBottom = [[-5.484, -3.784], [-5.484, -2.785], [-5.484, -0.761], [-5.484, 0.233], [-5.484, 2.188], [-5.484, 3.231]]
currentLocation = [0, 0]#get from odom 
orientation = []

focus = []  


class Robot_main:
	def __init__(self):
		# What to do if shut down (e.g. Ctrl-C or failure)
		rospy.on_shutdown(self.shutdown)

		self.bridge = CvBridge()

		self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info', 
			CameraInfo, self.camera_info_callback)

		self.cameraView = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",
			Image, self.image_callback)

		self.tf_listener = tf.TransformListener()

		self.goal_sent = False

		# What to do if shut down (e.g. Ctrl-C or failure)
		rospy.on_shutdown(self.shutdown)

		self.odom = rospy.Subscriber('/thorvald_001/odometry/base_raw', Odometry, self.odomCall) 
		
		# Tell the action client that we want to spin a thread by default
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait for the action server to come up")

		# Allow up to 5 seconds for the action server to come up
		self.move_base.wait_for_server(rospy.Duration(5))

		

	def camera_info_callback(self, data):#gets camera data 
		self.camera_model = image_geometry.PinholeCameraModel()
		self.camera_model.fromCameraInfo(data)		
		self.camera_info_sub.unregister() #Only subscribe once 
		global focus
		focus.extend(data.K)
		
		#odom for twist 
	def odomCall(self, msg):#get odometry data from robot 
		global currentLocation
		currentLocation[0] = msg.pose.pose.position.x
		currentLocation[1] = msg.pose.pose.position.y
		global orientation 
		orientation = msg.pose.pose.orientation
		

	  
	def image_callback(self, data):#main loop 
		#cv2.namedWindow("window", 1)
		same = False
		print("")
		print("")
		print ("start")
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		cv2.imshow("window", cv_image)
		hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		

		lower_green = numpy.array([44, 12, 30]) #bgr arrays 
		upper_green = numpy.array([80, 100, 70])
		mask = cv2.inRange(hsv_img, lower_green, upper_green)        
		mGre = cv2.moments(mask)
		
	
		mask = cv2.fastNlMeansDenoising(mask, None, 7, 7, 5) 
		#blur to remove some noise 
		mask = cv2.medianBlur(mask, 13)
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		
		#print how many objects have been found 
		print("I found {} objects".format(len(cnts)))
		for c in cnts:
		# calculate moments of binary image
			M = cv2.moments(c)
			#find centoid of object 
			if M["m10"] or M["m00"] or M["m01"] != 0:#error checking 
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				centoid = cX, cY
				#print'centoid: ', centoid 
				# highlight the centoid of each object found 
				cv2.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)

				p_robot = PoseStamped()
				p_robot.header.frame_id = "thorvald_001/kinect2_rgb_optical_frame"

				#camera coords 
				p_robot.pose.position.x  = ((cX - focus[2])*0.5)/focus[0]
				p_robot.pose.position.y = ((cY - focus[5])*0.5)/focus[4]			
				p_camera = self.tf_listener.transformPose('map', p_robot)
				#print("weed detected at x:" + str(p_camera.pose.position.x) + " y" + str(p_camera.pose.position.y))
				objectCoords[0] = p_camera.pose.position.x  
				objectCoords[1] = p_camera.pose.position.y 
				#print"test"
				objectsLocated.append(copy.copy(objectCoords))#its appending the same thing to all spaces in array 
				# print"objectsLocated ", objectsLocated
				# print"objectCoords ", objectCoords 

		
		cv_image = cv2.resize(cv_image, (0,0), fx=0.5, fy=0.5)
		mask = cv2.resize(mask, (0,0), fx=0.5, fy=0.5)
		cv2.imshow("hsv", mask)
		cv2.imshow("window", cv_image)
		#print'objectsLocated ', objectsLocated[0]
		#print'coordsBeen ', coordsBeen
		#time.sleep(20)
		if len(objectsLocated) != 0:
			#if objectsLocated[0] not in coordsBeen:
			#if not all(item in: coordsBeen for item in objectsLocated[0]):
			#print'len(coordsBeen)', len(coordsBeen)
			for t in coordsBeen:
				# print't', t
				# print'objectsLocated[0] ', objectsLocated[0]
				if objectsLocated[0] ==  t:
					same = True
					break
			if same == False:
				#print"entered"
				position = {'x':objectsLocated[0][0], 'y' : objectsLocated[0][1]}
				#print'position', position  
				quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}
					
				coordsBeen.append(objectsLocated[0])
				#print'coordsBeen ', coordsBeen				
				objectsLocated.pop(0)
				#print"objectsLocated ", objectsLocated[0]
				##------------------------------
				rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
				success = robot_main.goto(position, quaternion)

				if success:
					rospy.loginfo("Hooray, reached the desired pose")
					coordsBeen.append(copy.copy(objectsLocated[0]))
					print("spray weed")						#spray weed 
				else:
					rospy.loginfo("The base failed to reach the desired pose")
					#show images on screen 
					
						
			else:				
				objectsLocated.pop(0)
				#print"else"
				#print"objectsLocated ", objectsLocated[0]
				#print'coordsBeen ', coordsBeen		
		else:
			#print 'coordsTop: ', coordsTop
			#print 'coordsBottom: ', coordsBottom
			#print 'currentLocation: ', currentLocation
			if abs((coordsTop[0][0] - currentLocation[0]) + (coordsTop[0][1] - currentLocation[1])) < abs((coordsBottom[0][0] - currentLocation[0]) + (coordsBottom[0][1] - currentLocation[1])):
				position = {'x':coordsTop[0][0], 'y' : coordsTop[0][1]}
				quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}	
			else:
				position = {'x':coordsBottom[0][0], 'y' : coordsBottom[0][1]}
				quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

			coordsTop.pop(0)
			coordsBottom.pop(0)

			rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
			success = robot_main.goto(position, quaternion)

			if success:
				rospy.loginfo("Hooray, reached the desired pose")

				coordsBeen.append(copy.copy(objectsLocated[0]))
			else:
				rospy.loginfo("The base failed to reach the desired pose")
				#show images on screen 
			
				
		cv2.imshow("hsv", mask)
		cv2.imshow("window", cv_image)
		cv2.waitKey(3)


	def goto(self, pos, quat):
		# Send a goal
		self.goal_sent = True
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
		#print'target_pose', goal.target_pose.pose 
		# Start moving
		self.move_base.send_goal(goal)

		# Allow up to 60 seconds to complete task
		success = self.move_base.wait_for_result(rospy.Duration(30)) 

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
