#!/usr/bin/env python
import rospy
from xquad_ros.msg import ArucoArray
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
import tf
from tf.transformations import *

#OpenCV
import cv2
from cv_bridge import CvBridge


#RNA
import pickle
import numpy as np




'''
###//////////////////////////////////////////////////////////////////////###
###///////////////////////////PNP CODES//////////////////////////////////###
###//////////////////////////////////////////////////////////////////////###
'''

## Perspective-n-Point algorithm (PNP) - computes camera pose from gate beacons
def pose_estimation_gate(objp, imgp):
	global intrinsic_matrix, distortion_matrix

	ret, rvec, tvec = cv2.solvePnP(objp, imgp, intrinsic_matrix, distortion_matrix,flags=cv2.SOLVEPNP_ITERATIVE)


	rmat = np.zeros(shape=(3,3))
	rmat, _ = cv2.Rodrigues(rvec) 

	R_g_c = np.concatenate((rmat,tvec), axis=1 )

	
	H_g_c = np.concatenate((R_g_c,np.array([[0,0,0,1]])), axis = 0)

	# Flip from left to right hand frame
	R = np.array([[1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]])

	drone_rvec = rvec
	drone_tvec = tvec

	return H_g_c, ret, rvec, tvec




### Transform drone position with respect to the world
def transform_world(H_g_c):
	global curve_now, gates_seq
	
	gate_id = gates_seq[curve_now]
	gate_3d_position = rospy.get_param('/uav/'+gate_id+'/nominal_location',-1)
	gate_3d_position = np.array(gate_3d_position)

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

	return H_d_w



def getGateParametes(s_id):
	global gates_seq
	gates = {}
	for i in gates_seq:
		gate_name = i
		gate_location = rospy.get_param('/uav/'+gate_name+'/nominal_location',-1)
		if gate_location == -1:
			continue
		perturbation_bound = rospy.get_param('/uav/'+gate_name+'/perturbation_bound',-1)
		gates[gate_name] = {}
		gates[gate_name]['location'] = gate_location
		gates[gate_name]['perturbation_bound'] = perturbation_bound
	return gates


'''
###//////////////////////////////////////////////////////////////###
###//////////////////////////END PNP CODES///////////////////////###
###//////////////////////////////////////////////////////////////###
'''

# 
'''////////3D points/////////////'''
scalar = 3.3528
						# /* UP LEFT */ - corner 1 - x y
corners_3dp = np.array([[[-0.4329, -0.2955, -0.1/scalar],
						[-0.4329, -0.3301, -0.1/scalar],
						[-0.4329, -0.3647, -0.1/scalar],
						[-0.4329, -0.3983, -0.1/scalar],
						[-0.4329, -0.4329, -0.1/scalar],
						[-0.3983, -0.4329, -0.1/scalar],
						[-0.3647, -0.4329, -0.1/scalar],
						[-0.3301, -0.4329, -0.1/scalar],
						[-0.2955, -0.4329, -0.1/scalar],
						[-0.2619, -0.4329, -0.1/scalar]],
						# /* UP RIGHT */ - corner 2 - x y
						[[0.4318, -0.2955, -0.1/scalar],
						[0.4318, -0.3301, -0.1/scalar],
						[0.4318, -0.3647, -0.1/scalar],
						[0.4318, -0.3983, -0.1/scalar],
						[0.4318, -0.4329, -0.1/scalar],
						[0.3972, -0.4329, -0.1/scalar],
						[0.3636, -0.4329, -0.1/scalar],
						[0.3290, -0.4329, -0.1/scalar],
						[0.2944, -0.4329, -0.1/scalar],
						[0.2608, -0.4329, -0.1/scalar]],
						# /* DOWN RIGHT */ - corner 3 - x y
						[[0.4318, 0.2955, -0.1/scalar],
						[0.4318, 0.3290, -0.1/scalar],
						[0.4318, 0.3636, -0.1/scalar],
						[0.4318, 0.3972, -0.1/scalar],
						[0.4318, 0.4307, -0.1/scalar],
						[0.3972, 0.4307, -0.1/scalar],
						[0.3636, 0.4307, -0.1/scalar],
						[0.3290, 0.4307, -0.1/scalar],
						[0.2944, 0.4307, -0.1/scalar],
						[0.2608, 0.4307, -0.1/scalar]],
						# /* DOWN LEFT */ - corner 4 - x y
						[[-0.4329, 0.2955, -0.1/scalar],
						[-0.4329, 0.3290, -0.1/scalar],
						[-0.4329, 0.3636, -0.1/scalar],
						[-0.4329, 0.3972, -0.1/scalar],
						[-0.4329, 0.4307, -0.1/scalar],
						[-0.3983, 0.4307, -0.1/scalar],
						[-0.3647, 0.4307, -0.1/scalar],
						[-0.3301, 0.4307, -0.1/scalar],
						[-0.2955, 0.4307, -0.1/scalar],
						[-0.2619, 0.4307, -0.1/scalar]]])
corners_3dp = corners_3dp*scalar

#
#

Bigcorners_3dp = np.array([[-0.3636, -0.3636, -0.1/scalar],
							[0.3636, -0.3636, -0.1/scalar],
							[0.3636, 0.3636, -0.1/scalar],
							[-0.3636, 0.3636, -0.1/scalar]])
Bigcorners_3dp = Bigcorners_3dp*scalar


'''////////////////////////'''

def norm_scaler(X):
	n,m = X.shape
	Nv = m/2
	xmean = []
	Xn = np.zeros(X.shape)
	for i in range(n):
		m1 = np.mean(X[i,0:Nv])
		m2 = np.mean(X[i,Nv:m])
		xmean.append([m1, m2])
		Xn[i,0:Nv] = X[i,0:Nv] - m1
		Xn[i,Nv:m] = X[i,Nv:m] - m2

	for i in range(n):
		norm = 0
		for j in range(Nv):
			norm = norm + np.sqrt(Xn[i,j]**2 + Xn[i,j+Nv]**2)

		for j in range(m):
			Xn[i,j] = Xn[i,j]/(norm/Nv)

	return Xn



def identify_ids(points, model, id_corner):
	try:
		Xn = norm_scaler(points)
		IDS = np.round(model.predict(Xn)).astype(int)
		idx = IDS[0]
		# print(idx)
		objp = corners_3dp[id_corner][idx][:]
		return objp
	except:
		print("Waiting points")
	



def order_array(X, id):
	new_X = []
	new_Y = []
	orig_X = []
	orig_Y = []
	for i in range(len(X)):
		orig_X.append(X[i].x)
		orig_Y.append(X[i].y)
		new_X.append(X[i].x)
		new_Y.append(-X[i].y)

	orig_X = np.array(orig_X)
	orig_Y = np.array(orig_Y)
	new_X = np.array(new_X)
	new_Y = np.array(new_Y)

	if(id == 1 ):
		new_X = -new_X
	elif(id == 2):
		new_X = -new_X
		new_Y = -new_Y
	elif(id == 3):
		new_Y = -new_Y


	center_x = np.mean(new_X)
	center_y = np.mean(new_Y)

	ang = []
	for i in range(len(X)):
		aux = np.arctan2( (new_Y[i] - center_y) , (new_X[i] - center_x))
		ang.append((aux + 2 * np.pi) % (2 * np.pi))

	idx = np.argsort(-np.array(ang))

	## Points
	imgp = np.concatenate((orig_X[idx].reshape(-1,1),orig_Y[idx].reshape(-1,1)),axis=1)

	V = np.concatenate((new_X[idx],new_Y[idx]),axis=0)
	V = np.array(V)
	V = V.reshape(1,-1)

	return V, imgp


def draw_dots(data):
	global image
	if (image is None):
		return

	for k in range(len(data)):
		num_points = len(data[k].corners)
		for j in range(num_points):
			if (num_points == 10):
				cv2.circle(image, (int(data[k].corners[j].x),int(data[k].corners[j].y)), 2, (255,0,0),-1)
			else:
				if(data[k].id == 100):
					cv2.circle(image, (int(data[k].corners[j].x),int(data[k].corners[j].y)), 2, (0,0,255),-1)
				else:
					cv2.circle(image, (int(data[k].corners[j].x),int(data[k].corners[j].y)), 2, (0,255,255),-1)

	return image

def draw_square(img,objpts,rvec,tvec):
	global intrinsic_matrix, distortion_matrix, image

	imgpts, jac = cv2.projectPoints(objpts, rvec, tvec, intrinsic_matrix, distortion_matrix)
	for k in range(4):
		p1 = np.array(imgpts[k][0]).astype(int)
		p2 = np.array(imgpts[(k + 1) % 4][0]).astype(int)
		cv2.line(image, tuple(p1), tuple(p2), (0,255,0), 2)
	



def callback_corners(msg):
	global MLP_model, Bigcorners_3dp, pub_pose_beacons, pub_pose_raw, image

	if (len(MLP_model)<=0):
		return

	img = draw_dots(msg.aruco)

	imgp_total = []
	objp_total = []
	if (len(msg.aruco) >= 4):		
		for i in range(len(msg.aruco)):

			if(msg.aruco[i].id == 100):
				imgp = []
				for k in range(len(msg.aruco[i].corners)):
					imgp.append([msg.aruco[i].corners[k].x,msg.aruco[i].corners[k].y])

				imgp = np.array(imgp)
				objp = Bigcorners_3dp
			else:
				V, imgp = order_array(msg.aruco[i].corners, msg.aruco[i].id)
				if(len(msg.aruco[i].corners) == 10):
					objp = corners_3dp[msg.aruco[i].id]
				elif(len(msg.aruco[i].corners) == 9):
					objp = identify_ids(V,MLP_model[3],msg.aruco[i].id)
				elif(len(msg.aruco[i].corners) == 8):
					objp = identify_ids(V,MLP_model[2],msg.aruco[i].id)
				elif(len(msg.aruco[i].corners) == 7):
					objp = identify_ids(V,MLP_model[1],msg.aruco[i].id)
				elif(len(msg.aruco[i].corners) == 6):
					objp = identify_ids(V,MLP_model[0],msg.aruco[i].id)
				else:
					continue
			
			if (type(objp) is np.ndarray):
				imgp_total = imgp_total+imgp.tolist()
				objp_total = objp_total+objp.tolist()


	# PNP
	if (len(objp_total) > 14):
		H_g_c, ret, rvec, tvec = pose_estimation_gate(np.array(objp_total),np.array(imgp_total))
		H_d_w = transform_world(H_g_c)

		img = draw_square(img,Bigcorners_3dp, rvec, tvec)


		# Publish
		quat_d_w = quaternion_from_matrix(H_d_w)

		pose_beacons = PoseStamped()

		pose_beacons.header.stamp = rospy.get_rostime()
		pose_beacons.header.frame_id = "world"

		pose_beacons.pose.position.x = H_d_w[0][3]
		pose_beacons.pose.position.y = H_d_w[1][3]
		pose_beacons.pose.position.z = H_d_w[2][3]

		pose_beacons.pose.orientation.x = quat_d_w[0]
		pose_beacons.pose.orientation.y = quat_d_w[1]
		pose_beacons.pose.orientation.z = quat_d_w[2]
		pose_beacons.pose.orientation.w = quat_d_w[3]

		pub_pose_beacons.publish(pose_beacons)

	### Publish Raw Data
		quat_g_c = quaternion_from_matrix(H_g_c)

		pose_raw = PoseStamped()

		pose_raw.header.stamp = rospy.get_rostime()
		pose_raw.header.frame_id = "world"

		pose_raw.pose.position.x = H_g_c[0][3]
		pose_raw.pose.position.y = H_g_c[1][3]
		pose_raw.pose.position.z = H_g_c[2][3]

		pose_raw.pose.orientation.x = quat_g_c[0]
		pose_raw.pose.orientation.y = quat_g_c[1]
		pose_raw.pose.orientation.z = quat_g_c[2]
		pose_raw.pose.orientation.w = quat_g_c[3]

		pub_pose_raw.publish(pose_raw)

	
	cv2.imshow("detection", image)
	cv2.waitKey(1)


			



intrinsic_matrix = []
distortion_matrix = []
## Callback Camera Info
def get_camera_info(data):
	global intrinsic_matrix, distortion_matrix
	intrinsic_matrix = np.array([[data.K[0],data.K[1],data.K[2]],[data.K[3],data.K[4],data.K[5]],[data.K[6],data.K[7],data.K[8]]], dtype = "double")
	distortion_matrix = np.array([[data.D[0]],[data.D[1]],[data.D[2]],[data.D[3]],[data.D[4]]], dtype = "double")


image = []
def callback_image(data):
	global image
	bridge = CvBridge()
	image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')




curve_now = 0
def get_curve_now(data):
	global curve_now
	curve_now = data.data





MLP_model = []
def main():
	global MLP_model, gates_seq, pub_pose_beacons, pub_pose_raw
	rospy.init_node('detect_corner_ID', anonymous=True)
	rospy.Subscriber("/gate/corners_point", ArucoArray, callback_corners)
	rospy.Subscriber("/uav/camera/left/camera_info", CameraInfo, get_camera_info)
	rospy.Subscriber("/uav/camera/left/image_rect_color", Image, callback_image)
	rospy.Subscriber("/path/curve_now", Int32, get_curve_now)

	pub_pose_beacons = rospy.Publisher("/pose_beacons", PoseStamped, queue_size=1)
	pub_pose_raw = rospy.Publisher("/pose_beacons_raw", PoseStamped, queue_size=1)

	gates_seq = rospy.get_param('/uav/gate_names')

	# Load MLP
	filedir = rospy.get_param('/RNA_pnp/mlp_local')
	MLP_model_6 = pickle.load(open(filedir+"MLP_IDS_50k_6.sav", 'rb'))
	MLP_model_7 = pickle.load(open(filedir+"MLP_IDS_50k_7.sav", 'rb'))
	MLP_model_8 = pickle.load(open(filedir+"MLP_IDS_50k_8.sav", 'rb'))
	MLP_model_9 = pickle.load(open(filedir+"MLP_IDS_50k_9.sav", 'rb'))
	MLP_model = [MLP_model_6,MLP_model_7,MLP_model_8,MLP_model_9]
	rospy.spin()


########### MAIN #####################
if __name__ == '__main__':

	try:
		main()
	except rospy.ROSInterruptException:
		pass
