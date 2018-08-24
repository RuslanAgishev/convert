#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from math import *
import tf
from tf2_msgs.msg import TFMessage
import numpy as np
from pyquaternion import Quaternion

global data_to_publish
data_to_publish = PoseStamped()
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)


def cb_ar(data):
    br = tf.TransformBroadcaster()
    if not data.markers:
        print("no markers")
    else:
        for i in range(len(data.markers)):
            if data.markers[i].id == 0:
                data_to_publish.header = data.markers[i].header
                # get marker local position relative to the camera of the drone
                pose = data.markers[i].pose.pose
                cam_loc_pose = - np.array([pose.position.x,pose.position.y,pose.position.z])
                r = np.array([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]) # marker local rot
                camera_loc_rot = Quaternion( np.array([r[0],-r[1],-r[2],-r[3]]) / (np.linalg.norm(r)**2) )
                # compute transform to get camera global position:
                if marker_pos is not None:
                    camera_pos = marker_pos + cam_loc_pose
                    camera_rot = camera_loc_rot * Quaternion( marker_rot )
                    data_to_publish.pose.position.x = camera_pos[0]
                    data_to_publish.pose.position.y = camera_pos[1]
                    data_to_publish.pose.position.z = camera_pos[2]
                    data_to_publish.pose.orientation.x = camera_rot[0]
                    data_to_publish.pose.orientation.y = camera_rot[1]
                    data_to_publish.pose.orientation.z = camera_rot[2]
                    data_to_publish.pose.orientation.w = camera_rot[3]
                    pub.publish(data_to_publish)
                    #print("\npublished position data\n")
                    print data_to_publish

def quad_pos_from_marker():
    global marker_pos
    global marker_rot
    rospy.init_node('convert', anonymous=True)
    tf_listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, cb_ar)
        try:
            # get marker global position
            marker_pos, marker_rot = tf_listener.lookupTransform('/world', '/marker', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        rate.sleep()

if __name__ == '__main__':
    try:
        quad_pos_from_marker()
    except rospy.ROSInterruptException:
        pass
