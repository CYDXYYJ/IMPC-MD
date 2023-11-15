import numpy as np
from cvxopt import matrix, solvers
from cvxopt.solvers import options



def SVM(list1,list2,delta=0.0):

    D=len(list1[0])

    # list1是障碍物

    options.update({'show_progress':False,'abstol':1e-3,'reltol':1e-3}) 

    # list1 is the vertex list of obstacle

    n1=len(list1)
    n2=len(list2)
    
    G=np.zeros((n1+n2,D+1))
    h=np.zeros(n1+n2)

    
    G[0:n1,D]=1.0
    G[n1:n1+n2,D]=-1.0

    for i in range(0,n1):
        G[i][0:D]=list1[i]
    for i in range(n1,n1+n2):
        G[i][0:D]=-list2[i-n1]
        h[i]=-1
    
    P=np.eye(D+1)
    P[D][D]=0
    q=np.zeros(D+1)


    res = solvers.qp(P=matrix(P),q=matrix(q),G=matrix(G),h=matrix(h))
    
    a=res['x'][0:D]
    
    plane=(np.array(res['x'].T)[0])/np.linalg.norm(a)


    return plane