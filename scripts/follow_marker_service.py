#!/usr/bin/env python

## @package exp_assignment3
# \file follow_marker_service.py
# \brief This file contains code for 'follow_marker_server' node.
# \author Elham Mohammadi
# \version 1.0
# \date 25.01.2023
#
# \details
#
# Service: <BR>
# /request_follow_marker
#
# Publishes to: <BR>
#  /cmd_vel
#  
# Subscribes to: <BR> 
# /aruco_marker_id
# /camera2/image_raw/compressed
#
# This node waits for 'request_follow_marker' service's request from the 'simple_action' node to start the exploration behavior; 
# in which the robot starts rotating on its own axis until it sees aruco marker in its field of vision. Once a marker is detected, the
# robot starts moving closer to the marker until it is able to scan the marker. Once the marker is scaned and its 'id' has been retrive
# the exploration behavior stops and the node returns the marker's id as response to the service request. 
#


# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters
import math

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from exp_assignment3.srv import FollowMarker, FollowMarkerResponse
from std_msgs.msg import Int32

##  initializing global variable 'VERBOSE' with value 'False'.
VERBOSE = False

# service flag

# publishers  
##  initializing global variable 'vel_pub' for ROS topic 'cmd_vel'.
vel_pub_ = None 
##  initializing global variable 'goal_flag_'
goal_flag_ = True
##  initializing global variable 'np_arr_'
np_arr_ = None
##  initializing global variable 'marker_id' of type array which will be use to store marker ids during exploration mode (online).
marker_id = []
##  initializing global variable 'rsv_marker_id' of type array which will be use to store marker ids during offline mode.
rsv_marker_id = []



##
# \brief This is a callback function of 'camera2/image_raw/compressed' topic's subscriber.
# \return none
#
# This function recieves the compressed raw image of the environment captures through camera2 installed on the robot. 
#
def compressed_img_callback(ros_data):
    global np_arr_
    '''Callback function of subscribed topic. 
    Here images get converted and features detected'''
    if VERBOSE:
        print ('received image of type: "%s"' % ros_data.format)

    #### direct conversion to CV2 ####
    np_arr_ = np.frombuffer(ros_data.data, dtype=np.uint8)


##
# \brief The function checks if the provided id already exist or not in the 'marker_id' list. 
# 
# \return Bool
#
# The function checks if the provided id alreay exist or not in the 'marker_id' list which is the main list for storing markers id. 
#
def check_marker_ID(id):
    global marker_id
    for element in marker_id:
        if element == id:
            return False
    marker_id.insert(0,id)
    return True

##
# \brief The function checks if the provided id already exist or not in the 'rsv_marker_id' list. 
# 
# \return Bool
#
# The function checks if the provided id alreay exist or not in the 'rsv_marker_id' list which reserve list for storing markers id. 
#
def collect_reserve_marker_ID(id):
    global rsv_marker_id
    for element in rsv_marker_id:
        if element == id:
            return False
    rsv_marker_id.insert(0,id)
    return True

##
# \brief This is a callback function of 'aruco_marker_id' topic's subscriber.
#
# \return none
#
# This function recieves the aruco marker ids as they get scan by the two cameras installed on the robot's body. 
#
def aruco_marker_callback(msg):
    global goal_flag_
    if goal_flag_ == False:
        if msg.data >= 11 and msg.data <= 40 and check_marker_ID(msg.data):
            goal_flag_ = True
            print("Got the marker ID %d",msg.data)
    else:
        if msg.data >= 11 and msg.data <= 40 and collect_reserve_marker_ID(msg.data):
            print("Got the marker ID in reserve %d",msg.data)

##
# \brief This is a callback function of 'request_follow_marker' service.
# 
# \return FollowMarkerResponse returns the scaned marker id to simple_action client node. 
#
# The function implements the exploration behavior of the robot in which it detects aruco markers
# in its surronding. Once the marker is detected then through visual servoing it move closer to it
# in order to better scan it.
#
def callback_follow_marker(msg):
    global goal_flag_
    global rsv_marker_id
    goal_flag_ = False

    if(msg.req == "follow"):
        
        # check if reserve list is not empty
        # return the IDs from reserve list first then move forward.
        if(len(rsv_marker_id)>0):
            if(check_marker_ID(rsv_marker_id[0])):
                goal_flag_ = True
                response = FollowMarkerResponse()
                response.res = True
                response.markerid = rsv_marker_id[0]
                rsv_marker_id.pop(0)
                return response
            rsv_marker_id.pop(0)

        while not goal_flag_:

            image_np = cv2.imdecode(np_arr_, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
            
            whiteLower = (0, 0, 0)
            whiteUpper = (0, 0, 15)

            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, whiteLower, whiteUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            #cv2.imshow('mask', mask)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if radius > 7:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius),
                               (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = -0.002*(center[0]-400)
                    vel.linear.x = -0.01*(radius-100)
                    vel_pub_.publish(vel)
                else:
                    vel = Twist()
                    vel.linear.x = 1.0
                    vel_pub_.publish(vel)

                if(math.fabs(vel.linear.x) <= 0.05 and math.fabs(vel.angular.z) <= 0.01):
                    done()
            else:
                vel = Twist()
                vel.angular.z = 0.5
                vel_pub_.publish(vel)

            #cv2.imshow('window', image_np)
            cv2.waitKey(2)

        if(done()):
            print ("Follow marker task achieved.")
            goal_flag_ == True
            response = FollowMarkerResponse()
            response.res = True
            response.markerid = marker_id[0]
            return response        


##
# \brief This function reinstate robot's orientation and position for future exploration routine requests.
# 
# \return Bool
#
# The function first rotates the robot anti-clock wise for one second and then stop both its linear and angular velocities. 
# This is done so that in future request robot isn't facing towards the same aruco marker that it was before and start looking
# for new markers in the surrounding.
#
def done():
    twist_msg = Twist()
    twist_msg.angular.z = 1.0
    vel_pub_.publish(twist_msg)
    rospy.sleep(1.0)
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0
    vel_pub_.publish(twist_msg)
    return True


##
# \brief This is a 'main' function of oracle node. 
# 
# \return [none].
#
# This is a 'main' function of 'follow_marker_server' node. It initializes the 'request_follow_marker'
# service, a publisher for 'cmd_vel' topic and two clients for '/aruco_marker_id' and 'camera2/image_raw/compressed' topics
# 
# 
def main(args):
    global vel_pub_

    '''Initializes and cleanup ros node'''
    rospy.init_node('follow_marker_service')
    s = rospy.Service('request_follow_marker', FollowMarker, callback_follow_marker)
    vel_pub_ = rospy.Publisher("cmd_vel",Twist,queue_size=1)
    aruco_marker_ID_sub = rospy.Subscriber("/aruco_marker_id",Int32,aruco_marker_callback)
    compressed_img_subs = rospy.Subscriber("camera2/image_raw/compressed",CompressedImage,compressed_img_callback,queue_size=1)    
    print("follow_marker service is live...")     
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
