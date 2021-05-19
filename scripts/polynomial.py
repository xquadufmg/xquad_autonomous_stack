from numpy import *
from cvxopt import matrix, solvers


def polynomial(N,T0,T,C,Mat_eq,Mat_ineq):
    N = N+1


    Qr = zeros((N,N,N))
    Qr0 = zeros((N,N,N))


    for r in range(0, N):
        for i in range(0, N):
            for l in range(0, N):
                if(i>=r and l>=r):
                    Qr[r][i][l] = float(2*(T**(i+l-2*r+1)))/float(i+l-2*r+1)
                    for m in range(0, r):
                        Qr[r][i][l] = Qr[r][i][l] * (i-m)*(l-m)
                else:
                    Qr[r][i][l] = 0

        for i in range(0, N):
            for l in range(0, N):
                if(i>=r and l>=r):
                    Qr0[r][i][l] = float(2*(T0**(i+l-2*r+1)))/float(i+l-2*r+1)
                    for m in range(0, r):
                        Qr0[r][i][l] = Qr0[r][i][l] * (i-m)*(l-m)
                else:
                    Qr0[r][i][l] = 0

    Qr = Qr - Qr0
    #########################################

   
    ########  ########
    Q = 0
    for r in range(0, N):
        Q = Q+C[r]*Qr[r][:][:]
    #endFor
    #####################################


    ######  #########
    Ae = inf*ones((len(Mat_eq),N))
    Be = inf*ones((len(Mat_eq),1))

    for k in range(1,len(Mat_eq)+1,1):
        r = int(Mat_eq[k-1][0])  
        Be[k-1] = Mat_eq[k-1][2]
        for n in range(0, N):
            if(n >= r):
                product = 1
                for m in range(0, r):
                    product = product*(n-m)
                Ae[k-1][n] = product * Mat_eq[k-1][1]**(n-r)
            else:
                Ae[k-1][n] = 0
    
    ########################################

    ######  #########
    Ai = zeros((len(Mat_ineq),N))
    Bi = zeros((len(Mat_ineq),1))
    if Mat_ineq:
        Ai = inf*ones((len(Mat_ineq),N))
        Bi = inf*ones((len(Mat_ineq),1))
        for k in range(1, len(Mat_ineq)+1, 1):
            r = int(Mat_ineq[k-1][0]) 
            Bi[k-1] = Mat_ineq[k-1][2]
            for n in range(0, N):
                if(n >= r):
                    product = 1
                    for m in range(0, r):
                        product = product*(n-m)
                    Ai[k-1][n] = product * Mat_ineq[k-1][1]**(n-r)
                else:
                    Ai[k-1][n] = 0
            Ai[k-1][:] = Ai[k-1][:]*Mat_ineq[k-1][3]
            Bi[k-1] = Bi[k-1]*Mat_ineq[k-1][3]
    ###########################################

    ######  ##########
    p = zeros((1,N))

    Q = matrix(Q.tolist())
    p = matrix(p.tolist())
    Ai = matrix((Ai.transpose()).tolist())
    Bi = matrix((Bi.transpose()).tolist())
    Ae = matrix((Ae.transpose()).tolist())
    Be = matrix((Be.transpose()).tolist())

    Q = 0.5 * (Q + Q.T)

    solvers.options['show_progress'] = False
    P = solvers.qp(Q,p,Ai,Bi,Ae,Be)
    return P
    ################################################
