#!/usr/bin/python

import rospy
import tf
import tf.msg
import geometry_msgs.msg 
import numpy as np
from math import *

#Publisher to  /tf
def marker_position():
    rospy.init_node('convert', anonymous=True)
    br = tf.TransformBroadcaster()
    position = np.array([1,0,0])
    orientation = np.array(tf.transformations.quaternion_from_euler(0,0,radians(90)))
    while not rospy.is_shutdown():
        br.sendTransform(position, orientation, rospy.Time.now(), "marker", "world")
        print("published marker global position")
if __name__ == '__main__':
    try:
        marker_position()
    except rospy.ROSInterruptException:
        pass


