#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from math import *
from tf.transformations import *
import numpy as np

class Pose3D:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation
        
    def inv(self):
        '''
        Inversion of this Pose3D object
        
        :return inverse of self
        '''
        inv_rotation = self.rotation.transpose()
        inv_translation = -np.dot(self.rotation.transpose(), self.translation)
        return Pose3D(inv_rotation, inv_translation)
    
    def __mul__(self, other):
        '''
        Multiplication of two Pose3D objects, e.g.:
            a = Pose3D(...) # = self
            b = Pose3D(...) # = other
            c = a * b       # = return value
        
        :param other: Pose3D right hand side
        :return product of self and other
        '''
        return Pose3D(np.dot(self.rotation, other.rotation), np.dot(self.rotation, other.translation) + self.translation)
    
    def __str__(self):
        return "rotation:\n" + str(self.rotation) + "\ntranslation:\n" + str(self.translation.transpose())

def compute_quadrotor_pose(global_marker_pose, observed_marker_pose):
    '''
    :param global_marker_pose: Pose3D 
    :param observed_marker_pose: Pose3D
    
    :return global quadrotor pose computed from global_marker_pose and observed_marker_pose
    '''
    global_quadrotor_pose = global_marker_pose * observed_marker_pose.inv()
    return global_quadrotor_pose


def callback_ar(data):
    if not data.markers:
        print("no markers")
    else:
        # tracking of just one marker with the index 0
        q = np.zeros(4)
        q[0] = data.markers[0].pose.pose.orientation.x
        q[1] = data.markers[0].pose.pose.orientation.y
        q[2] = data.markers[0].pose.pose.orientation.z
        q[3] = data.markers[0].pose.pose.orientation.w
        t = np.zeros(3)
        t[0] = data.markers[0].pose.pose.position.x
        t[1] = data.markers[0].pose.pose.position.y
        t[2] = data.markers[0].pose.pose.position.z
        # creating of the object with observed marker coordinates from the webcam
        observed_marker_pose = Pose3D(quaternion_matrix(q)[:3,:3], t)
        # computing transformation with Pose3D objects
        global_quadrotor_pose = compute_quadrotor_pose(global_marker_pose, observed_marker_pose)
        # inverse transform from Pose3D object to PoseStamped ros-msg to publish data to /mavros/vision_pose/pose
        data_to_publish.header = data.markers[0].header
        R = identity_matrix()
        R[:3,:3] = global_quadrotor_pose.rotation
        q1 = quaternion_from_matrix(R)
        data_to_publish.pose.position.x = global_quadrotor_pose.translation[0]
        data_to_publish.pose.position.y = global_quadrotor_pose.translation[1]
        data_to_publish.pose.position.z = global_quadrotor_pose.translation[2]

        data_to_publish.pose.orientation.x = q1[0]
        data_to_publish.pose.orientation.y = q1[1]
        data_to_publish.pose.orientation.z = q1[2]
        data_to_publish.pose.orientation.w = q1[3]

        pub.publish(data_to_publish)
        print data_to_publish

def quad_pos_from_marker():
    # define global marker pose statically for now, assume that it is attached rigidly
    global global_marker_pose
    observed_marker_pose = None
    global_marker_pose = Pose3D(identity_matrix()[:3,:3],np.array([1,0,0]))
    rospy.init_node('convert', anonymous=True)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback_ar)
    rospy.spin()

global data_to_publish
data_to_publish = PoseStamped()
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
if __name__ == '__main__':
    try:
        quad_pos_from_marker()
    except rospy.ROSInterruptException:
        pass