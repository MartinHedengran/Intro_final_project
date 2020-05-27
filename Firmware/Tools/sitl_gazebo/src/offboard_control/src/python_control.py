#!/usr/bin/env python
# ROS python API
import rospy
import roslib
import sys
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import Image

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException as e:
    		print ("Service takeoff call failed: %s"%e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print ("Service disarming call failed: %s"%e)

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Stabilized Mode could not be set."%e)

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Altitude Mode could not be set."%e)

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Position Mode could not be set."%e)

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)
    
        
class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 20.0
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP
        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 20.0)

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0

        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.parameters =  cv2.aruco.DetectorParameters_create()

        self.cx = 320/2
        self.cy = 240/2
        self.fx = 277.191356 # This is found by a rostopic 
        self.fy = 277.191356

        self.cameraMatrix = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=np.float)
        self.distCoeffs = np.array([[0, 0, 0, 0]], dtype=np.float)
        self.cameraTimer = 0
        self.landingFlag = False

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

	# Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    # def updateSp(self):
    #     self.sp.position.x = self.local_pos.x
    #     self.sp.position.y = self.local_pos.y

    def x_dir(self):
        self.sp.position.x = self.local_pos.x + 5
    	self.sp.position.y = self.local_pos.y
        
    def arUco_detection(self,image):
        if self.cameraTimer < 250:
            self.cameraTimer += 1
        else:
            self.cameraTimer = 0
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            except CvBridgeError as e:
                print(e)
            
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(cv_image, self.dictionary, parameters=self.parameters)
            frame_markers = cv2.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds) # image.copy()
            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.05, self.cameraMatrix, self.distCoeffs)

            #The position isn't needed. Just the notation to get the specific element out it importen here
            x = tvecs[0][0,0]/0.05
            y = tvecs[0][0,1]/0.05
            z = tvecs[0][0,2]/0.05

            # Draw circle on center of image
            #y_center = image.shape[0]/2
            #x_center = image.shape[1]/2
            #cv2.circle(image,(x_center,y_center),5,self.color,2)

            # y_diff = (y)*self.calculate_GSD(self.altitude) # x and y are flipped due to the placement of the camera
            # x_diff = (x)*self.calculate_GSD(self.altitude)

            # self.point_msg.header.stamp = rospy.Time.now()
            # self.point_msg.pose.position.x = -y
            # self.point_msg.pose.position.y = -x
            self.sp.position.x = self.local_pos.x - y
            self.sp.position.y = self.local_pos.y - x

            if (-0.1 < x < 0.1 and -0.1 < y < 0.1 and self.landingFlag == True):
                self.sp.position.z = 0

            elif (-0.5 < x < 0.5 and -0.5 < y < 0.5):
                self.sp.position.z = 5
                self.landingFlag = True


            qvecs = self.euler_to_quaternion(rvecs[0][0,0], 0, 0)

            # self.point_msg.pose.orientation.x = qvecs[0]
            # self.point_msg.pose.orientation.y = qvecs[1]
            # self.point_msg.pose.orientation.z = qvecs[2]
            # self.point_msg.pose.orientation.w = qvecs[3]

            print('y_diff= ',y)
            print('x_diff= ',x)

            #print('rvecs= ',rvecs[0])

            # Rotation 180deg around the x-axis for the acuro marker.
            #rvecs_flipped = rvecs[0].dot(self.R_flip)   

            #print(rvecs_flipped)

            # for rvec, tvec in zip(rvecs_flipped, tvecs[0]):
            #     frame_markers = cv2.aruco.drawAxis(frame_markers, self.cameraMatrix,
            #             self.distCoeffs, rvec, tvec, 0.05)

    def calculate_GSD(self, a):
        focal = 1           #Length in cm
        im_w = 752          #Width in pixels
        FOV = 1.3962634     #Angle in radians
        altitude = a*100    #Altitude in cm

        sensor_w = math.tan(FOV/2)*focal*2
        GSD = 0.01*(altitude*sensor_w)/(focal*im_w) # m/px
        print(GSD)

        return GSD

    def euler_to_quaternion(self, yaw, pitch, roll):

            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

            return [qx, qy, qz, qw]
    def neg_x_dir(self):
    	self.sp.position.x = self.local_pos.x - 5
    	self.sp.position.y = self.local_pos.y

    def y_dir(self):
    	self.sp.position.x = self.local_pos.x
    	self.sp.position.y = self.local_pos.y + 5

    def neg_y_dir(self):
    	self.sp.position.x = self.local_pos.x
    	self.sp.position.y = self.local_pos.y - 5


# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    rospy.Subscriber("/iris_fpv_cam/usb_cam/image_raw", Image, cnt.arUco_detection)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)


    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    # set in takeoff mode and takeoff to default altitude (3 m)
    # modes.setTakeoff()
    # rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()

    # ROS main loop
    while not rospy.is_shutdown():
    	#cnt.updateSp()
    	sp_pub.publish(cnt.sp)
    	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass