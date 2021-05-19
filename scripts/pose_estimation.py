#!/usr/bin/python
import rospy
import time
from math import sqrt
import numpy as np
import cv2
import yaml
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation

import tf

from flightgoggles.msg import IRMarkerArray
from flightgoggles.msg import IRMarker
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import CameraInfo
from tf.transformations import *
from std_msgs.msg import Int32
import geometry_msgs.msg as g_msg
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry


import copy


class PoseEstimation:

	def __init__(self):

		# Variables
		self.gates = self.getGateParametes()  
		self.intrinsic_matrix = []
		self.distortion_matrix = []
		self.gates_seq = rospy.get_param('/uav/gate_names')
		self.index_gate_seq = 0
		self.drone_tvec = []
		self.drone_rvec = []
		self.last_markers = {}
		self.init_pose = rospy.get_param('/uav/flightgoggles_uav_dynamics/init_pose')
		self.last_beacons = [[],[]]
		self.last_trans = self.init_pose[0:3]
		self.last_vel = [0,0,0]
		self.last_time = rospy.get_rostime()
		self.ekf_odom = Odometry()
		self.curve_now	= 0
		self.trans_last = [0, 0, 0]
		# Size of intern gate
		self.Width = 2.5/2.0
		self.Height = 2.5/2.0
		# label for pnp
		self.new_pnp = False
		# PNP result
		self.H_g_c = []


		# Publisher
		self.pub_pose_beacons = rospy.Publisher("/pose_beacons", g_msg.PoseStamped, queue_size=1)
		self.pub_pose_raw = rospy.Publisher("/pose_beacons_raw", g_msg.PoseStamped, queue_size=1)

		# Subscriber
		rospy.Subscriber("/uav/camera/left/camera_info", CameraInfo, self.get_camera_info)
		time.sleep(0.3)
		rospy.Subscriber("/path/curve_now", Int32, self.get_curve_now)
		# rospy.Subscriber("/uav/camera/left/ir_beacons_proc", IRMarkerArray, self.callback_beacons)
		rospy.Subscriber("/uav/camera/left/ir_beacons", IRMarkerArray, self.callback_beacons)
		


	## Callback Camera Info
	def get_camera_info(self, data):
		self.intrinsic_matrix = np.array([[data.K[0],data.K[1],data.K[2]],[data.K[3],data.K[4],data.K[5]],[data.K[6],data.K[7],data.K[8]]], dtype = "double")
		self.distortion_matrix = np.array([[data.D[0]],[data.D[1]],[data.D[2]],[data.D[3]],[data.D[4]]], dtype = "double")


	## Get Gates Info from Parameters
	def getGateParametes(self):
		gates_id = rospy.get_param('/uav/gate_names')
		gates = {}
		for i in gates_id:
			# gate_name = 'Gate' + str(i)
			gate_name = i
			gate_location = rospy.get_param('/uav/'+gate_name+'/nominal_location',-1)
			if gate_location == -1:
				continue
			perturbation_bound = rospy.get_param('/uav/'+gate_name+'/perturbation_bound',-1)
			gates[gate_name] = {}
			gates[gate_name]['location'] = gate_location
			gates[gate_name]['perturbation_bound'] = perturbation_bound
		return gates


	## Callback Curve_now (from control script)
	def get_curve_now(self, data):
		self.curve_now = data.data



#============= Callback IR-Beacons position ============================
	def callback_beacons(self, data):
		# gate ID to use (based on curve_now)
		gate_id = self.gates_seq[self.curve_now]
		# gate_id = "Gate1"	# MIGUE P RODAR SEM FLIGHTGOGGLES

		n_gates = len(data.markers)/4

		gates = {}
		for i in range(0,n_gates):
			landmark_IDX = i
			for j in range(0,4):
				landmark_id_ = data.markers[j + 4*i].landmarkID.data
				marker_id_	 = int(data.markers[j + 4*i].markerID.data)
				try:
					gates[landmark_IDX][landmark_id_][marker_id_] = (data.markers[j + 4*i].x, data.markers[j + 4*i].y)
				except:
					gates[landmark_IDX] = {}
					gates[landmark_IDX][landmark_id_] = {}
					gates[landmark_IDX][landmark_id_][marker_id_] = (data.markers[j + 4*i].x, data.markers[j + 4*i].y)
		
		# Min num ir beacons to execute PNP algorithm
		min_num_pts = 4


		real_dist = {}
		min_dist = 1000000000000000
		H_g_c = {}
		idx = []
		ret = 0
		for gate in gates.keys():
			real_dist[gate] = {}
			# # Verification of all detected ports

			print(gates[gate])

			for landmark in gates[gate].keys():

				# If it is not the correct gate, pass
				if( landmark != gate_id ):
					continue

				print(gate)
				# Checking the number of beacons
				# print(len(gates[gate][landmark].keys()))
				if (len(gates[gate][landmark].keys()) >= min_num_pts):

					num_pts = len(gates[gate][landmark].keys())

					gate_image_position = []
					gate_3d_position_all = {}
					gate_3d_position_all[gate] = {}
					gate_3d_position_world = []
					gate_3d_position = np.array([[-self.Width,-self.Height, 0.0], [self.Width,-self.Height, 0.0], [self.Width,self.Height, 0.0], [-self.Width,self.Height, 0.0]])


					for identifiers in gates[gate][landmark].keys():
						gate_image_position.append( gates[gate][landmark][identifiers] )
						gate_3d_position_world.append( self.gates[landmark]['location'][identifiers-1] ) # MIGUE P RODAR SEM FLIGHTGOGGLES


					# Nummpy array transform
					gate_image_position = np.float64(gate_image_position)
					gate_3d_position_all[gate] = np.array(gate_3d_position_world) # MIGUE P RODAR SEM FLIGHTGOGGLES
					# gate_3d_position_all[gate] = np.array([[3.54902, 28.27437, 2.815452], [1.11062, 28.27437, 2.815452], [1.11062, 28.27437, 0.377048], [3.54902, 28.27437, 0.377048]])


					# Pose of camera with respect to the gate frame
					H_g_c[gate], ret = self.pose_estimation_gate( gate_3d_position, gate_image_position)


			if ret:
				real_dist[gate] = self.estimate_distance_gate(H_g_c[gate])
				if(real_dist[gate] < min_dist):
					min_dist = real_dist[gate]
					self.H_g_c = H_g_c[gate]
					idx = gate

		if ret:		
		### Transformations and publish
			# print p_g_w
			print(self.H_g_c)

			H_d_w = self.transform_world(self.H_g_c, gate_3d_position_all[idx])

			quat_d_w = quaternion_from_matrix(H_d_w)


			pose_beacons = g_msg.PoseStamped()

			pose_beacons.header.stamp = rospy.get_rostime()
			pose_beacons.header.frame_id = "world"

			pose_beacons.pose.position.x = H_d_w[0][3]
			pose_beacons.pose.position.y = H_d_w[1][3]
			pose_beacons.pose.position.z = H_d_w[2][3]

			pose_beacons.pose.orientation.x = quat_d_w[0]
			pose_beacons.pose.orientation.y = quat_d_w[1]
			pose_beacons.pose.orientation.z = quat_d_w[2]
			pose_beacons.pose.orientation.w = quat_d_w[3]

			self.pub_pose_beacons.publish(pose_beacons)

		### Publish Raw Data
			quat_g_c = quaternion_from_matrix(self.H_g_c)
			# print ("Quat = ", quat_g_c)
			# print ("Pos = ",[self.H_g_c[0][3],self.H_g_c[1][3],self.H_g_c[2][3]])

			pose_raw = g_msg.PoseStamped()

			pose_raw.header.stamp = rospy.get_rostime()
			pose_raw.header.frame_id = "world"

			pose_raw.pose.position.x = self.H_g_c[0][3]
			pose_raw.pose.position.y = self.H_g_c[1][3]
			pose_raw.pose.position.z = self.H_g_c[2][3]

			pose_raw.pose.orientation.x = quat_g_c[0]
			pose_raw.pose.orientation.y = quat_g_c[1]
			pose_raw.pose.orientation.z = quat_g_c[2]
			pose_raw.pose.orientation.w = quat_g_c[3]

			self.pub_pose_raw.publish(pose_raw)


#======================================================



	## Perspective-n-Point algorithm (PNP) - computes camera pose from gate beacons
	def pose_estimation_gate(self, objp, imgp):

		# self.intrinsic_matrix = []
		# self.distortion_matrix = []
		n, m = imgp.shape

		if n == 3:
			objp = np.concatenate( ( objp, [objp[2]] ), axis=0)
			imgp = np.concatenate( ( imgp, [imgp[2]] ), axis=0)


		# Computes the PNP
		if len(self.drone_tvec) == 0:
			ret, rvec, tvec = cv2.solvePnP(objp, imgp, self.intrinsic_matrix, self.distortion_matrix,
									flags=cv2.SOLVEPNP_ITERATIVE)
		else:
			ret, rvec, tvec = cv2.solvePnP(objp, imgp, self.intrinsic_matrix, self.distortion_matrix,
										self.drone_rvec, self.drone_tvec, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE )



		rmat = np.zeros(shape=(3,3))
		rmat, _ = cv2.Rodrigues(rvec) 

		R_g_c = np.concatenate((rmat,tvec), axis=1 )

		
		H_g_c = np.concatenate((R_g_c,np.array([[0,0,0,1]])), axis = 0)

		# Flip from left to right hand frame
		R = np.array([[1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]])
		# H_g_c = R.dot(H_g_c)

		self.drone_rvec = rvec
		self.drone_tvec = tvec

		return H_g_c, ret



	### Distance to the gate
	def estimate_distance_gate(self, H_g_c):
		x = H_g_c[0][3]
		y = H_g_c[1][3]
		z = H_g_c[2][3]
		
		distance = sqrt(x**2 +	y**2 + z**2)
		return distance



	### Transform drone position with respect to the world
	def transform_world(self, H_g_c, gate_3d_position):

		p_g_w = np.array([np.mean(gate_3d_position[:,0]),np.mean(gate_3d_position[:,1]),np.mean(gate_3d_position[:,2])])
		v_cima = np.array([0,0,1])
		v_direita = gate_3d_position[1,:] - gate_3d_position[0,:]
		v_cima = v_cima/np.linalg.norm(v_cima)
		v_direita = v_direita/np.linalg.norm(v_direita)
		v_in = np.cross(v_cima,v_direita)
		theta = np.arctan2(v_in[1],v_in[0])
		R = np.array([[np.cos(theta), -np.sin(theta), 0.0],[np.sin(theta), np.cos(theta), 0],[0,0,1]])

		R_pos = np.concatenate((R,np.array([[p_g_w[0]],[p_g_w[1]],[p_g_w[2]]])), axis=1 )


		H_g0_w = np.concatenate((R_pos,np.array([[0,0,0,1]])), axis = 0)

		H_g_g0 = np.array([[0.0,0.0,1.0,0.0],[-1.0,0.0,0.0,0.0],[0.0,-1.0,0.0,0.0],[0.0,0.0,0.0,1.0]])

		# Left camera is always at center of mass of drone. Right camera is offset by 32cm. (Using left)
		H_c_d = np.array([[0.0,0.0,1.0,0.0],[-1.0,0.0,0.0,0.0],[0.0,-1.0,0.0,0.0],[0.0,0.0,0.0,1.0]])


		H_g_w = H_g0_w.dot(H_g_g0)
		H_c_w = H_g_w.dot(np.linalg.inv(H_g_c))
		H_d_w = H_c_w.dot(np.linalg.inv(H_c_d))

		# H_g_w = H_g2_w.dot(H_g_g2)
		# H_c_w = H_g_w.dot(np.linalg.inv(H_g_c))
		# # H_c_w = H_g_w.dot(H_g_c)
		# H_d_w = H_c_w.dot(np.linalg.inv(H_c_d))

		return H_d_w





def main():

	rospy.init_node("pose_estimation2")

	pose_estimation = PoseEstimation()

	rate = rospy.Rate(60.0)

	pub_test = rospy.Publisher("/uav/camera/left/ir_beacons_proc", IRMarkerArray, queue_size=1)
	msg = IRMarkerArray()
	while not rospy.is_shutdown():	
		msg.markers = []	
		msg.header.stamp = rospy.get_rostime()
		# data = IRMarker()
		# data.landmarkID.data = "Gate1"
		# data.markerID.data = "1"
		# data.x = 200.0
		# data.y = 100.0
		# data.z = 0.0
		# msg.markers.append(data)
		# #
		# data = IRMarker()
		# data.landmarkID.data = "Gate1"
		# data.markerID.data = "2"
		# data.x = 310.0
		# data.y = 100.0
		# data.z = 0.0
		# msg.markers.append(data)
		# #
		# data = IRMarker()
		# data.landmarkID.data = "Gate1"
		# data.markerID.data = "3"
		# data.x = 310.0
		# data.y = 210.0
		# data.z = 0.0
		# msg.markers.append(data)
		# #
		# data = IRMarker()
		# data.landmarkID.data = "Gate1"
		# data.markerID.data = "4"
		# data.x = 200.0
		# data.y = 210.0
		# data.z = 0.0
		# msg.markers.append(data)


		## GT x = 2.0, y = 36.0, z = 1.5
		## Quaternion ->  0.0 0.0 -0.7071 0.7071
		#############################
		data = IRMarker()
		data.landmarkID.data = "Gate1"
		data.markerID.data = "1"
		data.x = 248.0
		data.y = 179.0
		data.z = 0.0
		msg.markers.append(data)
		#
		data = IRMarker()
		data.landmarkID.data = "Gate1"
		data.markerID.data = "2"
		data.x = 357.0
		data.y = 179.0
		data.z = 0.0
		msg.markers.append(data)
		#
		data = IRMarker()
		data.landmarkID.data = "Gate1"
		data.markerID.data = "3"
		data.x = 358.0
		data.y = 288.0
		data.z = 0.0
		msg.markers.append(data)
		#
		data = IRMarker()
		data.landmarkID.data = "Gate1"
		data.markerID.data = "4"
		data.x = 249.0
		data.y = 286.0
		data.z = 0.0
		msg.markers.append(data)

		################

		# print(msg)
		# pub_test.publish(msg)

		rate.sleep()


if __name__ == '__main__':

		try:
				main()
		except rospy.ROSInterruptException:
				pass
