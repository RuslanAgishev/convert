#!/usr/bin/env python


# #############   Subscriber

# import rospy
# from std_msgs.msg import String
# from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

# def callback(data):
#     # rospy.loginfo('I heard %s', data)
	
#     if not data.markers:
#         print "no markers"
#     else:
#         print "\nposition data:\n"
#         # print data.markers[0].pose.pose
#         print data.markers[0].pose

# def listener():

#     rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
#     rospy.spin()

# if __name__ == '__main__':
#     listener()



# #############   Publisher

# import rospy
# from std_msgs.msg import String

# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         #rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass


###############   UNITED

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from math import *
import tf

data_to_publish = PoseStamped()
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

def callback(data):
	global data_to_publish
	if not data.markers:
		print "no markers"

	else:
		print "\npublished position data:\n"
	

		# print data.markers[0].pose
		# print "header:"
		# print data.markers[0].header
		# print "pose:"
		# print data.markers[0].pose.pose

		# print data.markers[0].pose.pose.orientation.
		# print "number of markers: ", len(data.markers)

		for i in range(len(data.markers)):
			# print "id:", data.markers[i].id, "type:", type(data.markers[i].id)
			if data.markers[i].id == 0:

				# data_to_publish = data.markers[0].pose
				data_to_publish.header = data.markers[i].header #take main heading with time stamp
				data_to_publish.pose = data.markers[i].pose.pose
				quaternion0 = (
				    data_to_publish.pose.orientation.x,
				    data_to_publish.pose.orientation.y,
				    data_to_publish.pose.orientation.z,
				    data_to_publish.pose.orientation.w
				    )
				euler = tf.transformations.euler_from_quaternion(quaternion0)
				roll = euler[0]
				pitch = euler[1]
				yaw = euler[2]
				#print(yaw)

				data_to_publish.pose.position.z = -  data_to_publish.pose.position.z + 3.90 #to get ENU frame coordinate
				data_to_publish.pose.position.y = - data_to_publish.pose.position.y #to get ENU frame coordinate
				data_to_publish.pose.position.x = data_to_publish.pose.position.x

				print(data_to_publish.pose.position)

				quaternion0 = (
				    data_to_publish.pose.orientation.x,
				    data_to_publish.pose.orientation.y,
				    data_to_publish.pose.orientation.z,
				    data_to_publish.pose.orientation.w
				    )
				euler = tf.transformations.euler_from_quaternion(quaternion0)
				roll = euler[0]
				pitch = euler[1]
				yaw = euler[2]
				print(yaw)

				quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
				data_to_publish.pose.orientation.x = quaternion[0]
				data_to_publish.pose.orientation.y = quaternion[1]
				data_to_publish.pose.orientation.z = quaternion[2]
				data_to_publish.pose.orientation.w = quaternion[3]			

				#print data_to_publish
				
				pub.publish(data_to_publish)

		# data.markers[0].pose.pose.position.x
		# data.markers[0].pose.pose.position.y
		# data.markers[0].pose.pose.position.z

		# q_x = data.markers[0].pose.pose.orientation.x
		# q_y = data.markers[0].pose.pose.orientation.y
		# q_z = data.markers[0].pose.pose.orientation.z
		# q_w = data.markers[0].pose.pose.orientation.w
		# yaw = atan2(2.0*(q_y*q_z + q_w*q_x), q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z)
		# print yaw

		# print data.markers[0].pose.pose.orientation.x

def ar2pixhawk():
	rospy.init_node('convert', anonymous=True)
	rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		ar2pixhawk()
	except rospy.ROSInterruptException:
		pass
