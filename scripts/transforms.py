#!/usr/bin/env python
import rospy

import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import quaternion_matrix, euler_matrix, quaternion_from_matrix
import numpy as np





class Orb_Transform:
	def __init__(self):

		rospy.init_node('orb_transform', anonymous=True)

		self.New_pose = PoseStamped()

		self.pub_pose = rospy.Publisher('/orbslam2/new_pose', PoseStamped, queue_size=1)
		rospy.Subscriber("/orb_slam2_stereo/pose", PoseStamped, self.orb2)
		rospy.spin()




	def orb2(self, msg):
		R = quaternion_matrix([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
		T = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
		R[0][3]= T[0]
		R[1][3]= T[1]
		R[2][3]= T[2]
		# H_can_map = np.block([[R, T],[0, 0, 0, 1]])
		A = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
		R_2 = quaternion_matrix([A[3],A[4],A[5],A[6]])
		# MIGUE
		# R_2 = quaternion_matrix([-0.0177, -0.0177, -0.7069, 0.7069])
		R_2[0][3]= A[0]
		R_2[1][3]= A[1]
		R_2[2][3]= A[2]

		H = R_2.dot(R)

		
		self.New_pose.header.stamp = rospy.get_rostime()
		self.New_pose.header.frame_id = "world"
		self.New_pose.pose.position.x = H[0][3]
		self.New_pose.pose.position.y = H[1][3]
		self.New_pose.pose.position.z = H[2][3]
		quaternion = quaternion_from_matrix(H)
		self.New_pose.pose.orientation.x = quaternion[0]
		self.New_pose.pose.orientation.y = quaternion[1]
		self.New_pose.pose.orientation.z = quaternion[2]
		self.New_pose.pose.orientation.w = quaternion[3]

		self.pub_pose.publish(self.New_pose)


	# # def run(self):
	# 	rate = rospy.Rate(50)
	# 	while not rospy.is_shutdown():



########### MAIN #####################
if __name__ == '__main__':
	try:
		node = Orb_Transform()

	except rospy.ROSInterruptException:
		pass
