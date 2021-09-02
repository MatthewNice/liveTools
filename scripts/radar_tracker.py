#!/usr/bin/env python

###this is going to be the radar tracker node###

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import sys
import numpy as np
sys.path.insert(0,'/home/circles/catkin_ws/src/live_radar')
sys.path
import kalmanTracking as k



gmyDetections = []

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
def callback384(data):
    global gmyDetections
    gmyDetections.append([data.point.x, data.point.y,data.point.z])
#    rospy.loginfo(data.point.x)
def callback385(data):
    global gmyDetections
    gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback386(data):
	global gmyDetections
	gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback387(data):
	global gmyDetections
	gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback388(data):
	global gmyDetections
	gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback389(data):
    global gmyDetections
    gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback390(data):
    global gmyDetections
    gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback391(data):
    global gmyDetections
    gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback392(data):
    global gmyDetections
    gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback393(data):
    global gmyDetections
    gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback394(data):
    global gmyDetections
    gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback395(data):
    global gmyDetections
    gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback396(data):
    global gmyDetections
    gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback397(data):
    global gmyDetections
    gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback398(data):
    global gmyDetections
    gmyDetections.append([data.point.x, data.point.y,data.point.z])
def callback399(data):
    global gmyDetections
    gmyDetections.append([data.point.x, data.point.y,data.point.z])


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
	rospy.init_node('radar_tracker', anonymous=True)
    #radar messages
	rospy.Subscriber("/track_a0", PointStamped, callback384)
	rospy.Subscriber("/track_a1", PointStamped, callback385)
	rospy.Subscriber("/track_a2", PointStamped, callback386)
	rospy.Subscriber("/track_a3", PointStamped, callback387)
	rospy.Subscriber("/track_a4", PointStamped, callback388)
	rospy.Subscriber("/track_a5", PointStamped, callback389)
	rospy.Subscriber("/track_a6", PointStamped, callback390)
	rospy.Subscriber("/track_a7", PointStamped, callback391)
	rospy.Subscriber("/track_a8", PointStamped, callback392)
	rospy.Subscriber("/track_a9", PointStamped, callback393)
	rospy.Subscriber("/track_a10",PointStamped, callback394)
	rospy.Subscriber("/track_a11",PointStamped, callback395)
	rospy.Subscriber("/track_a12",PointStamped, callback396)
	rospy.Subscriber("/track_a13",PointStamped, callback397)
	rospy.Subscriber("/track_a14",PointStamped, callback398)
	rospy.Subscriber("/track_a15",PointStamped, callback399)

# spin() simply keeps python from exiting until this node is stopped
#	rospy.spin()

if __name__ == '__main__':
    listener()
    r = rospy.Rate(20)
    detections = np.random.rand(1,3)
    tracker = k.KF_Tracker(0.05)
    tracked_objects = tracker(detections)

    while not rospy.is_shutdown():
#        rospy.loginfo(gmyDetections)
        tracked_objects = tracker(gmyDetections)
        rospy.loginfo(tracked_objects)
        gmyDetections = []
        r.sleep() 
  # rospy.loginfo(gmyDetections)
