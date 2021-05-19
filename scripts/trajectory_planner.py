#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
from numpy import *
from numpy import linalg as LA
from polynomial import polynomial
from planning.msg import Coefs



#------------------------ Gates Selection --------------------------------------#
gates_id = rospy.get_param('/uav/gate_names')
initial_pos = rospy.get_param('/uav/flightgoggles_uav_dynamics/init_pose')

N = 10                                          # Polynomial Order
T0 = 0                                          # Initial time
V_DESIRED_0 = 10.0*1.0                          # Initial Desired Velocity
C = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]           # Cost vector
CI_P0 = initial_pos[0:3]                        # Initial position
CI_V0 = [0,3.0,2.0]                             # Initial velocity

## Desloc traj (MIGUE)
CI_P0[1] = CI_P0[1] + 2

DIST_IN = 1.5
DIST_OUT = 0.5
DP = 1.0
DV = 1.0
SMOOTH_FACTOR = 1.0
MAX_ACC = 30.0

SIGMA = 0.7
GAMMA = 1.0
ALPHA = pi/3.5      # Aproximated Camera Appearture


SHIFT_Z = -0


#--------------------------------#----------------------------------------------#


#------------------------- Class Windows ---------------------------------------#
class Windows():
    def __init__(self,gates_id):
        global N
        self.pub_coefs = rospy.Publisher("/planner/coefs", Coefs, queue_size=1)

        self.v_desired_vec = zeros(len(gates_id))            # Desired Velocity Gate
        self.flag_received = [False for k in range(len(gates_id))]

        self.corners = zeros((len(gates_id),4,3))            # Gates Corners Position
        self.centers = zeros((len(gates_id),3))               # Gates Center Position
        self.vect = zeros((len(gates_id),3))
        self.area_gate = ones((len(gates_id),1))
        self.length_path = ones((len(gates_id),1))

        for k in range(0, len(gates_id)):
            string = '/uav/'+str(gates_id[k])+'/nominal_location'
            x = matrix(rospy.get_param(string), dtype='f')
            self.corners[k,:,:] = x
            normal_vec = cross((self.corners[k,0,:]-self.corners[k,1,:]),(self.corners[k,1,:] - self.corners[k,2,:]))
            self.area_gate[k] = LA.norm(normal_vec)
            normal_vec = normal_vec/(self.area_gate[k])
            self.vect[k] = normal_vec

            # Compute the center of the gates
            for i in range(0,3):
                self.centers[k,i] = mean(self.corners[k,:,i])
        # Check if the normal vectors are pointing in the proper directions
        for k in range(0, len(gates_id)):
            if k==0:
                if(dot((self.centers[k,:]-array(CI_P0).transpose()),self.vect[k,:]) < 0):
                    self.vect[k,:] = -self.vect[k,:]
            else:
                if(dot((self.centers[k,:]-self.centers[k-1,:]),self.vect[k,:]) < 0):
                    self.vect[k,:] = -self.vect[k,:]

            if k == 1 and gates_id[0:2] == ['Gate10', 'Gate21']:
                self.vect[k] = (1-SIGMA)*self.vect[k] + SIGMA*array([0.0,0.9511,0.3090])
                self.vect[k] = self.vect[k]/LA.norm(self.vect[k])


#-------------- Heuristics for the path --------------------------------------#
    def heuristic(self):
        for k in range(0,len(self.centers)):
            if k == 0:
                self.v_desired_vec[k] = V_DESIRED_0
            elif k == len(self.centers)-1:
                self.v_desired_vec[k] = V_DESIRED_0
            else:
                k_n = k+1
                k_p = k-1

                D1 = LA.norm(self.centers[k_p]-self.centers[k])
                D2 = LA.norm(self.centers[k]-self.centers[k_n])

                minD = min([D1,D2])

                heu = (minD/16.0)**0.7
                if heu > 1:
                    heu = 1

                self.v_desired_vec[k] = V_DESIRED_0*heu
        return


    ###################################################### ######################################################
    ###################################################### ######################################################
    ###################################################### ######################################################
    def heuristic_GAMMA(self):
        vec_cross_dir = matrix(zeros((3,len(self.centers))))
        for k in range(0,len(self.centers)):
            if k == 0:
                self.v_desired_vec[k] = V_DESIRED_0
            elif k == len(self.centers)-1:
                self.v_desired_vec[k] = V_DESIRED_0
            else:
                k_n = k+1
                k_p = k-1

                D1 = LA.norm(self.centers[k_p]-self.centers[k])
                D2 = LA.norm(self.centers[k]-self.centers[k_n])

                minD = min([D1,D2])

                heu = (minD/16.0)**0.7
                if heu > 1:
                    heu = 1

                self.v_desired_vec[k] = V_DESIRED_0*heu


                a = D1/(D1+D2)
                A = matrix([[0, 0, 0, 0, 1],
                    [1, 1, 1, 1, 1],
                    [a**4, a**3, a**2, a, 1],
                    [0, 0, 0, 1, 0],
                    [4, 3, 2, 1, 0]])
                B = matrix([self.centers[k_p,:].tolist(),self.centers[k_n,:].tolist(),self.centers[k,:].tolist(),dot(self.vect[k_p,:].tolist(),(D1+D2)),dot(self.vect[k_n,:].tolist(),(D1+D2))])
                C = dot(LA.inv(A),B)
                vec_cross_dir[:,k] = dot(C.transpose(),matrix([(4*a**3).tolist(), (3*a**2).tolist(), (2*a).tolist(), 1, 0]).transpose())
                vec_cross_dir[:,k] = vec_cross_dir[:,k]/LA.norm(vec_cross_dir[:,k])

        for k in range(1,len(self.vect)-1):
            if k == 1 and gates_id[0:2] == ['Gate10', 'Gate21']:
                self.vect[k] = (1-SIGMA)*self.vect[k] + SIGMA*array([0.0,0.9511,0.3090/3.0])
                self.vect[k] = self.vect[k]/LA.norm(self.vect[k])
            else:
                if k == 3:
                    self.vect[k,:] = (1-GAMMA/10)*self.vect[k,:] + (GAMMA/10)*vec_cross_dir[:,k].transpose()
                    self.vect[k] = self.vect[k]/LA.norm(self.vect[k])
                else:
                    self.vect[k,:] = (1-GAMMA)*self.vect[k,:] + GAMMA*vec_cross_dir[:,k].transpose()
                    self.vect[k] = self.vect[k]/LA.norm(self.vect[k])
        return


    # Computation of a simple spline in order to approximate the length of the path
    def trajectory_heuristics(self, T, p0, pf, v0, vf):
        A = matrix([[0, 0, 0, 1],[T**3, T**2, T, 1],[0, 0, 1, 0],[3*T**2, 2*T, 1, 0]])
        B = matrix([p0.tolist(),pf.tolist(),v0.tolist(),vf.tolist()])
        c = dot(LA.inv(A),B)
        t = linspace(0,T,100)
        vels = dot(c.transpose(),matrix([(3*t**2).tolist(), (2*t).tolist(), ones((1,len(t))).tolist()[0], zeros((1,len(t))).tolist()[0]]))
        vels = array(vels.tolist())
        norm_vel = sqrt(vels[0,:]**2+vels[1,:]**2+vels[2,:]**2)
        l = norm_vel.sum()*T/len(t)

        # Compute the integral of the curvature alonge the length_path
        dt = t[1]-t[0]
        int_curv = 0
        for k in range(1,len(vels[1,:]),1):
            vec_1 = vels[:,k]/LA.norm(vels[:,k])
            vec_0 = vels[:,k-1]/LA.norm(vels[:,k-1])
            int_curv = int_curv + LA.norm(vec_1-vec_0)

        int_curv = int_curv**0.25

        return l, int_curv




    #---------- Trajectory Planning ----------#
    def coefs(self):
        coefs_T = zeros((len(self.centers)))
        coefs_cx = matrix(zeros((len(self.centers),len(self.centers))))
        coefs_cy = matrix(zeros((len(self.centers),len(self.centers))))
        coefs_cz = matrix(zeros((len(self.centers),len(self.centers))))

        for k in range(0,len(self.centers)):

            # Center Displacement to Up
            self.centers[k,:] = self.centers[k,:] + dot([0.0,0.0,0.17],amax(self.corners[k,:,2])-amin(self.corners[k,:,2]))

            v_desired = self.v_desired_vec[k]
            if k == 0:
                k_ant = len(self.centers)-1
                p0 = CI_P0
                v0 = CI_V0
                p_in = self.centers[k]-array([0,0,SHIFT_Z])-self.vect[k]*DIST_IN

                p_out = p0
            else:
                k_ant = k-1
                p0 = self.centers[k_ant]-array([0,0,SHIFT_Z])
                v0 = self.vect[k_ant]*self.v_desired_vec[k_ant]
                p_in = self.centers[k]-array([0,0,SHIFT_Z])-self.vect[k]*DIST_IN
                p_out = self.centers[k_ant]-array([0,0,SHIFT_Z])+self.vect[k_ant]*DIST_OUT


            v_desired_med = (self.v_desired_vec[k_ant]+self.v_desired_vec[k])/2.0

            heuristic_path = DIST_OUT + LA.norm(p_in-p_out) + DIST_IN
            T = T0 + SMOOTH_FACTOR*heuristic_path/v_desired_med
            time_d_in = T0 + ((heuristic_path-DIST_IN)/heuristic_path)*(T-T0)
            time_d_out = T0 + ((DIST_OUT)/heuristic_path)*(T-T0)
            vf = self.vect[k]*self.v_desired_vec[k]


            l, int_curv = self.trajectory_heuristics(T, array(p0).transpose(), self.centers[k,:], array(v0).transpose(), vf)
            #Redefinition of T according to the new spline heuristics
            T = T0 + SMOOTH_FACTOR*l/v_desired_med
            #Penalization of T according to the integral of the curvature of the path
            time_d_in = T0 + ((l-DIST_IN)/l)*(T-T0)
            time_d_out = T0 + ((DIST_OUT)/l)*(T-T0)

            # Matrices of equality constraints
            Mat_eq_x = [[0.0, T0, p0[0]],
                        [0.0, T, self.centers[k,0]],
                        [1.0, T0, v0[0]],
                        [1.0, T, vf[0]]]
            Mat_eq_y = [[0.0, T0, p0[1]],
                        [0.0, T, self.centers[k,1]],
                        [1.0, T0, v0[1]],
                        [1.0, T, vf[1]]]
            Mat_eq_z = [[0.0, T0, p0[2]],
                        [0.0, T, self.centers[k,2]-SHIFT_Z],
                        [1.0, T0, v0[2]],
                        [1.0, T, vf[2]]]

            # Matrices of inequality constraints
            Mat_ineq_x = [[0.0, time_d_in, p_in[0]+DP, 1.0],
                        [0.0, time_d_out, p_out[0]+DP, 1.0],
                        [1.0, time_d_in, vf[0]+DV, 1.0],
                        [1.0, time_d_out, v0[0]+DV, 1.0],
                        [0.0, time_d_in, p_in[0]-DP, -1.0],
                        [0.0, time_d_out, p_out[0]-DP, -1.0],
                        [1.0, time_d_in, vf[0]-DV, -1.0],
                        [1.0, time_d_out, v0[0]-DV, -1.0]]
            Mat_ineq_y = [[0.0, time_d_in, p_in[1]+DP, 1.0],
                        [0.0, time_d_out, p_out[1]+DP, 1.0],
                        [1.0, time_d_in, vf[1]+DV, 1.0],
                        [1.0, time_d_out, v0[1]+DV, 1.0],
                        [0.0, time_d_in, p_in[1]-DP, -1.0],
                        [0.0, time_d_out, p_out[1]-DP, -1.0],
                        [1.0, time_d_in, vf[1]-DV, -1.0],
                        [1.0, time_d_out, v0[1]-DV, -1.0]]
            Mat_ineq_z = [[0.0, time_d_in, p_in[2]+DP, 1.0],
                        [0.0, time_d_out, p_out[2]+DP, 1.0],
                        [1.0, time_d_in, vf[2]+DV, 1.0],
                        [1.0, time_d_out, v0[2]+DV, 1.0],
                        [0.0, time_d_in, p_in[2]-DP, -1.0],
                        [0.0, time_d_out, p_out[2]-DP, -1.0],
                        [1.0, time_d_in, vf[2]-DV, -1.0],
                        [1.0, time_d_out, v0[2]-DV, -1.0]]


            if k == 2:
                for tt in linspace(T0,T,100):
                    concatenate((Mat_ineq_x,[[0.0, tt, 21.0, 1.0]]))
                    concatenate((Mat_ineq_x,[[0.0, tt, -20.0, -1.0]]))
            P_x = polynomial(N,T0,T,C,Mat_eq_x,Mat_ineq_x)
            P_x = P_x['x']
            P_y = polynomial(N,T0,T,C,Mat_eq_y,Mat_ineq_y)
            P_y = P_y['x']
            P_z = polynomial(N,T0,T,C,Mat_eq_z,Mat_ineq_z)
            P_z = P_z['x']

            self.length_path[k] = 0.0

            for j in range(0,100):
                t = j*T/101.0 + 0.5*T/100.0
                v = [0.0,0.0,0.0]

                for i in range(1,11):
                    v[0] += i*P_x[i]*t**(i-1)
                    v[1] += i*P_y[i]*t**(i-1)
                    v[2] += i*P_z[i]*t**(i-1)

                self.length_path[k] += sqrt(v[0]**2+v[1]**2+v[2]**2)*(T/100.0)


            coefs_msg = Coefs()
            coefs_msg.curve_id = k
            coefs_msg.T = T
            coefs_msg.cx = array(P_x).transpose().tolist()[0]
            coefs_msg.cy = array(P_y).transpose().tolist()[0]
            coefs_msg.cz = array(P_z).transpose().tolist()[0]
            coefs_msg.fh = (1.0/tan(ALPHA))*(sqrt(self.area_gate[k])/self.length_path[k])[0]
            if k == 10:
                coefs_msg.fh = 0.0


            while(not self.flag_received[k]):
                self.flag_received[k] = True
                self.pub_coefs.publish(coefs_msg)


        return

#--------------------------------#----------------------------------------------#


#------------------------------ Main -------------------------------------------#

if __name__ == '__main__':
    rospy.init_node("trajectory_planner")
    rate = rospy.Rate(10.0)
    WINDOWS = Windows(gates_id)
    sleep(1)
    WINDOWS.heuristic()
    WINDOWS.coefs()
    sleep(5)

#--------------------------------#----------------------------------------------#
#--------------------------------#----------------------------------------------#
