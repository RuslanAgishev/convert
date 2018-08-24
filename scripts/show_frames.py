#!/usr/bin/env python  
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from math import *
import tf


def ar_cb(data):
    br_ar = tf.TransformBroadcaster()
    for i in range(len(data.markers)):
        br_ar.sendTransform((data.markers[i].pose.pose.position.x,data.markers[i].pose.pose.position.y,data.markers[i].pose.pose.position.z),
                     (data.markers[i].pose.pose.orientation.x,data.markers[i].pose.pose.orientation.y,data.markers[i].pose.pose.orientation.z,data.markers[i].pose.pose.orientation.w),
                     rospy.Time.now(),
                     "marker",
                     "world")

def locpos_cb(lp):
    br_lp = tf.TransformBroadcaster()
    br_lp.sendTransform((lp.pose.position.x,lp.pose.position.y,lp.pose.position.z),
                     (lp.pose.orientation.x,lp.pose.orientation.y,lp.pose.orientation.z,lp.pose.orientation.w),
                     rospy.Time.now(),
                     "quadrotor",
                     "world")

def vispos_cb(vp):
    br_vp = tf.TransformBroadcaster()
    br_vp.sendTransform((vp.pose.position.x,vp.pose.position.y,vp.pose.position.z),
                     (vp.pose.orientation.x,vp.pose.orientation.y,vp.pose.orientation.z,vp.pose.orientation.w),
                     rospy.Time.now(),
                     "visual_pos",
                     "world")

if __name__ == '__main__':
    rospy.init_node('convert')
    ar_marker_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_cb)
    local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, locpos_cb)
    local_pos_sub = rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, vispos_cb)
    rospy.spin()
