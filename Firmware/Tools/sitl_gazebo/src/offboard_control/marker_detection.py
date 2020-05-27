#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import math
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge, CvBridgeError

from MarkerLocator.MarkerTracker import MarkerTracker

class Nfold_detection_node:
    def __init__(self):
        rospy.init_node('marker_locater')
        self.image_pub = rospy.Publisher("locater_image1321241847162", Image, queue_size=10)
        self.pose_pub = rospy.Publisher("position_dif", PoseStamped, queue_size=10)

        self.bridge = CvBridge()
        self.altitude = 0
        self.color = 0
        self.point_msg = PoseStamped()

        rospy.Subscriber("/hummingbird/camera/camera_/image_raw", Image, self.callback)
        rospy.Subscriber("/hummingbird/ground_truth/position", PointStamped, self.call_pose)
        
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.parameters =  cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX   # python -m pip install opencv-contrib-python  ---- The command that is needed for this
        
        #parameter for insintric parameter(camera matrix)
        self.cx = 752/2
        self.cy = 480/2
        self.fx = 448.1 # This is found by a rostopic 
        self.fy = 448.1
        # The camera matrix and distortion coefficients
        self.cameraMatrix = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=np.float)
        self.distCoeffs = np.array([[0, 0, 0, 0]], dtype=np.float)

        #--- FLipping 180 deg around the x-axis for the aRuco-marker axis so it has the same direction as the camera.
        self.R_flip = np.zeros((3,3))
        self.R_flip[0,0] = 1.0
        self.R_flip[1,1] = -1.0
        self.R_flip[2,2] = -1.0




    def callback(self, data):
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.altitude > 12:
            cv_image=self.nFold_detection(cv_image)
        else:
            try:
                cv_image=self.arUco_detection(cv_image)
            except Exception as e:
                print(e," -- Could not see ArUca marker")

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))#bgr8 mono8
        except CvBridgeError as e:
            print(e)
        except Exception as e:
            print(e)
        
        try:
            self.pose_pub.publish(self.point_msg)
        except Exception as e:
            print(e)
    
    def call_pose(self, data):
        try:
            self.altitude = data.point.z
        except Exception as e:
            print(e)
        
    def nFold_detection(self, image):
        
        tracker = MarkerTracker(
            order=3, 
            kernel_size=30,     #30
            scale_factor=3.5) # scale_factor=31.5
        tracker.track_marker_with_missing_black_leg = False

        # Scale down the image and convert it to grayscale
        #image = cv2.resize(image, None, fx=0.4, fy=0.4)
        grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Locate the marker
        pose = tracker.locate_marker(grayscale_image)

        # Extract the marker response from the tracker object
        magnitude = np.sqrt(tracker.frame_sum_squared)

        # Getting the centerpoint in the image
        y_center = grayscale_image.shape[0]/2
        x_center = grayscale_image.shape[1]/2

        x_diff = (y_center-pose.y)*self.calculate_GSD(self.altitude) # x and y are flipped due to the placement of the camera
        y_diff = (x_center-pose.x)*self.calculate_GSD(self.altitude)

        self.point_msg.header.stamp = rospy.Time.now()
        self.point_msg.pose.position.x = x_diff
        self.point_msg.pose.position.y = y_diff

        print('y_diff= ',y_diff)
        print('x_diff= ',x_diff)

        # Visualise the location of the located marker and indicate the quality
        # of the detected marker by altering the line self.color.
        kvaliti=255 - int(255 * pose.quality)
        self.color = (0,kvaliti,kvaliti)
        cv2.line(image, (0, 0), (pose.x, pose.y), self.color, 2)
        cv2.circle(image,(x_center,y_center),5,self.color,2)
        #print(pose.quality)
        
        #return x_diff, y_diff   
        return image

    def arUco_detection(self,image):
        
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(image, self.dictionary, parameters=self.parameters)
        frame_markers = cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds) # image.copy()
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.05, self.cameraMatrix, self.distCoeffs)

        #The position isn't needed. Just the notation to get the specific element out it importen here
        x = tvecs[0][0,0]/0.05
        y = tvecs[0][0,1]/0.05
        z = tvecs[0][0,2]/0.05

        # Draw circle on center of image
        y_center = image.shape[0]/2
        x_center = image.shape[1]/2
        cv2.circle(image,(x_center,y_center),5,self.color,2)

        # y_diff = (y)*self.calculate_GSD(self.altitude) # x and y are flipped due to the placement of the camera
        # x_diff = (x)*self.calculate_GSD(self.altitude)

        self.point_msg.header.stamp = rospy.Time.now()
        self.point_msg.pose.position.x = -y
        self.point_msg.pose.position.y = -x

        qvecs = self.euler_to_quaternion(rvecs[0][0,0], 0, 0)

        self.point_msg.pose.orientation.x = qvecs[0]
        self.point_msg.pose.orientation.y = qvecs[1]
        self.point_msg.pose.orientation.z = qvecs[2]
        self.point_msg.pose.orientation.w = qvecs[3]

        print('y_diff= ',y)
        print('x_diff= ',x)

        #print('rvecs= ',rvecs[0])

        # Rotation 180deg around the x-axis for the acuro marker.
        rvecs_flipped = rvecs[0].dot(self.R_flip)   

        print(rvecs_flipped)

        for rvec, tvec in zip(rvecs_flipped, tvecs[0]):
            frame_markers = cv2.aruco.drawAxis(frame_markers, self.cameraMatrix,
                    self.distCoeffs, rvec, tvec, 0.05)

        return frame_markers

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
def main(args):
    ic = Nfold_detection_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)