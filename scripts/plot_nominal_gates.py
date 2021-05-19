#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np



global freq
freq = 1.1

global gates_pose
gates_pose = []


def read_gates_locations():

	gates_pose = []

	# Melhor iterar sobre rosparam com os nomes dos gates
	for k in range(9):

		my_str = "/uav/Gate"+str(k+1)+"/nominal_location"

		try:
			# print my_str
			x = rospy.get_param(my_str)
			# print x, " ", type(x),"\n\n"
			p1 = np.matrix(x[0])
			p2 = np.matrix(x[1])
			p3 = np.matrix(x[2])
			p4 = np.matrix(x[3])

			center = (p1+p2+p3+p4)/4;

			normal_vec = np.cross(p1-p2, p2-p3)
			area_gate = np.linalg.norm(normal_vec)
			normal_vec = normal_vec/area_gate


			if(abs(normal_vec[0,2]) < 0.001):

				theta = atan2(normal_vec[0,1],normal_vec[0,0])

				R = np.matrix([[cos(theta),-sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]])

				x = np.matrix(x)
				
				x[:,0] = x[:,0] - center[0,0]
				x[:,1] = x[:,1] - center[0,1]
				x[:,2] = x[:,2] - center[0,2]
				x = x.T
				# print "\33[92m x: "+str(x)+"\33[0m"
				x = R.T*x

				max_corner = max(x.tolist())
				min_corner = min(x.tolist())

				delta_x = 2*(max_corner[0]-min_corner[0])
				delta_y = 2*(max_corner[1]-min_corner[1])
				delta_z = 2*(max_corner[2]-min_corner[2])

				gates_pose.append([center[0,0], center[0,1], center[0,2], theta+0*3.1416/2.0, delta_x, delta_y, delta_z ])

			else:
				print "\33[93mSkipping the plot of gate "+str(k+1)+"\33[0m"


		except:
			print "\33[41mProblem occurred when trying to read the parameters!: plot_nominal_gates.py\33[0m"


	return gates_pose

# ----------  ----------  ----------  ----------  ----------




# Rotina executada apenas uma vez para mostrar a ellipse no rviz
def send_gates_to_rviz(gates_pose):

	points_marker = MarkerArray()
	marker = Marker()
	for k in range(len(gates_pose)):
		x = gates_pose[k][0]
		y = gates_pose[k][1]
		z = gates_pose[k][2]
		theta = gates_pose[k][3]
		delta_x = gates_pose[k][4]
		delta_y = gates_pose[k][5]
		delta_z = gates_pose[k][6]
		#print 'z = ', z
		marker = Marker()
		marker.header.frame_id = "/world"
		marker.header.stamp = rospy.Time.now()
		marker.id = k
		marker.type = marker.CUBE
		marker.action = marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = delta_x
		marker.scale.z = delta_z
		marker.color.a = 0.2
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.pose.orientation.w = cos(theta/2.0)
		marker.pose.orientation.z = sin(theta/2.0)
		marker.pose.position.x = x
		marker.pose.position.y = y
		marker.pose.position.z = z
		#print "marker = ", marker
		points_marker.markers.append(marker)

	return (points_marker)
# ----------  ----------  ----------  ----------  ----------






# Rotina primaria
def just_plot():
	global freq


	vel = Twist()

	i = 0

	rospy.init_node("just_plot")

	pub_gates = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=1) #rviz array de marcadores no espaco da elipse

	rate = rospy.Rate(freq)

	gates_pose = read_gates_locations()

	pointsMarker = send_gates_to_rviz(gates_pose)

	sleep(1)

	while not rospy.is_shutdown():

		i = i + 1
		time = i / float(freq)


		pub_gates.publish(pointsMarker)

		rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------






# Funcao inicial
if __name__ == '__main__':

	try:
		just_plot()
	except rospy.ROSInterruptException:
		pass
