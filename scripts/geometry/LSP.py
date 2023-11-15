import numpy as np
from cvxopt import matrix, solvers
from cvxopt.solvers import options


def linear_separting_plane(list1,list2):

    # list1 是障碍物

    options.update({'show_progress':False,'abstol':1e-3,'reltol':1e-3}) 

    D=len(list1[0])

    num=len(list1)
    h=len(list2)
    
    
    A=np.zeros((h+num+2*D+2,D+1))
    b=np.zeros(h+num+2*D+2)
    
    A[0:num,D]=1
    A[num:h+num,D]=-1
    for i in range(0,num):
        A[i][0:D]=list1[i]
    for i in range(num,num+h):
        A[i][0:D]=-list2[i-num]

    # 这一部分是用来限制变量范围的
    A[num+h:num+h+D+1,0:D+1]=np.eye(D+1)
    A[num+h+D+1:num+h+2*D+2,0:D+1]=-np.eye(D+1)
    
    b[num+h:num+h+D+1]=np.array([1,1,1,100])
    b[num+h+D+1:num+h+2*D+2]=np.array([1,1,1,100])

    c=np.zeros(D+1)
    c[D]=-1
    c[0:D]=-list2[-1] # 最后一个点离分割面的距离尽量的远
    
    res =solvers.lp(matrix(c),matrix(A),matrix(b))
    a=res['x'][0:D]
    
    plane=(np.array(res['x'].T)[0])/np.linalg.norm(a)
    
    return plane 