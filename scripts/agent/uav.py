import numpy as np
from scipy import linalg as lg
import sys 
sys.path.append("..")
from geometry.geometry3d import detect_line_collision,line,polygon,detect_polygon_collision
import copy
from cvxopt import matrix, solvers
from cvxopt.solvers import options


class uav():


    # initialization
    def __init__(self,
        index,
        type,
        K,
        h,
        D,
        ini_p,
        Num,                # the number of agent need to be avioded
        REALFLY=False,
        obstacle_list=[],
        path=[],
        buffer=0.1,
        buffer_connect=0.1,
        neighbour=[],
        height=0.5):


        ##############################
        ####### key cofficient #######
        ##############################

        # the index of this agent 
        self.index=index

        # the type of this UAV:
        self.type=type

        # get the neighbour:
        self.neighbor=neighbour
        self.neighbor.sort()

        # the length of horizon
        self.K=K

        self.h=h

        self.D=D

        self.buffer=buffer

        self.buffer_connect=buffer_connect

        self.REALFLY=REALFLY

        #####################
        ####### state #######
        #####################

        # initial position
        self.ini_p=ini_p.copy()

        ini_v=np.zeros(self.D)

        # current position            
        self.p=ini_p.copy()

        # current velocity
        self.v=ini_v.copy()


        # current state including position and velocity 
        self.state=np.append(self.p,self.v)

        # maximum acc
        self.Umax=3.0

        # maximum velocity
        self.Vmax=15.0

        # the diameter of a robot (pay attention: it is diameter!)
        self.r_min=4.0

        self.r_max=self.Vmax*self.h*self.K

        # dynamic matrix
        self.get_dynamic()

        self.get_coef_matrix()

        ##########################
        ########## map ###########
        ##########################

        self.obstacle_list=copy.deepcopy(obstacle_list)

        ##########################
        ####### Trajectory #######
        ##########################

        # target position
        if self.type == "Searcher":
            self.target = path[-1].copy()
            self.path = np.block([[self.ini_p],[path]])
        else:
            self.target=self.p.copy()
            self.path = None
        

        # terminal position
        self.terminal_p=self.p.copy()


        # a coefficient related to the objective
        self.cost_index=K


        # the predetermined trajectory
        self.pre_traj=np.zeros((self.K+1,self.D))

        for i in range(self.K+1):
            self.pre_traj[i]=self.p.copy()


        # tractive position
        self.tractive_point=None

        if self.type == "Searcher":

            self.tractive_point = self.ini_p.copy()

        # the tractive position list for objective
        self.G_p=None

        # get tractive point for obstace transitor
        self.get_tractive_point()

        
        # the list of all time's 
        self.pre_traj_list=[]

        # the list of all past position
        self.position=self.p.copy()



        #######################
        ####### Dealock #######
        #######################

        self.rho_0=200.0

        # warning band width
        self.epsilon=3.0

        self.term_overlap=False

        self.term_last_p=self.p.copy()

        self.E=0.0

        self.term_overlap_again=False

        self.term_last_p=self.p.copy()

        self.term_index=0

        self.eta=1.0

        self.E=self.epsilon/2*np.ones(Num-1)

        self.W=self.epsilon/2*np.ones(Num)


        #########################
        ####### crazyfile #######
        #########################
        
        if self.REALFLY:

            # the height of crazyfile is constantly 1m
            self.height=height

            # the yaw of crazyfile is constantly 0.0 deg
            self.yaw=0.0

            # the input trajectory
            self.input_traj=self.get_input_traj()
        
        ###############################
        ####### data collection #######
        ###############################
        
        self.data=np.zeros(self.D)
        self.data[0:2]=np.array([self.index,self.D])
        self.data=np.block([[self.data],\
            [self.ini_p],[self.target],[-9999999*np.ones(self.D)]])
        

    # run convex program of each agents
    def run_convex_program(self):

        type=self.type
        
        ####### functional constraints related #######
        inter_cons_A = self.inter_cons_A
        inter_cons_B = self.inter_cons_B
        inter_cons_C = self.inter_cons_C
        # ob_cons_A = self.ob_cons_A
        # ob_cons_B = self.ob_cons_B
        ob_cons_A = np.zeros(self.D*self.K)
        ob_cons_B = -1*np.ones(1)
        group_cons_A=self.group_cons_A
        group_cons_B=self.group_cons_B
        connection_constraint_list=self.connection_constraint_list
        Rho_ij = self.Rho_ij
        epsilon = self.epsilon
        buffer=self.buffer
        buffer_connect=self.buffer


        ####### dynamic related #######

        state = self.state

        K = self.K 
        D = self.D

        VA=self.VA
        VB = self.VB
        VC=self.VC

        Umax = self.Umax
        Vmax = self.Vmax

        Xi = self.Xi
        Phi = self.Phi
        Xi_K = self.Xi_K

        G_p = self.G_p
        W = self.W.copy()

        ######## objective function related #######

        # related to warning band
        M_log=Rho_ij/W/2/epsilon
        M_log=np.diag(M_log) 

        if np.linalg.norm(self.terminal_p-self.target) > 20.0:
            Q_tar = 400.0*2.0/np.linalg.norm(self.terminal_p-self.target)
        else:
            Q_tar = 400.0

        if type == "Free-transitor" or type == "Obstacle-transitor" or type == "Searcher":

            cost_index=self.cost_index

            # get the needed weight coefficient matrix
            Sigma=np.zeros([ D* K, D* K])
            
            for i in range(max([cost_index-1,0]), K):
                for j in range( D):
                    Sigma[ D*i+j][ D*i+j]=Q_tar

            Delta_P = 1.0*self.Delta_P


            Q= VB.T @  Phi.T @ Sigma @  Phi @  VB  +  VB.T @  Phi.T @ Delta_P @  Phi @  VB
            p = 2* VB.T @  Phi.T @ Sigma @ (  Phi @  ( VA @ state + VC ) - G_p) +\
                2* VB.T @  Phi.T @ Delta_P @ Phi @  ( VA @ state + VC ) 

        elif type == "Connector" or type == "Subleder":

            P_neighbor = self.P_neighbor.copy()

            Q = 5*VB.T @  Phi.T @  Phi @  VB
            p = 10*VB.T @  Phi.T @ (Phi @  ( VA @ state + VC ) - P_neighbor) 
        
        ##############################
        ####### convex program #######
        ##############################
        
        # variables
        len_U=D*K
        len_E=inter_cons_C.shape[1]
        len_ob_B=len(ob_cons_B)
        len_group_B=len(group_cons_B)
        l=len_U+len_E+len_ob_B+len_group_B

        # if exists connection constriants, adds variables 
        if connection_constraint_list:
            len_conn=len(connection_constraint_list)*K
            l+=len_conn


        P = lg.block_diag(Q,M_log,200*np.eye(len_ob_B),200*np.eye(len_group_B))
        q = np.block([p,-2*M_log @ np.ones(len_E)*epsilon,np.zeros(len_ob_B),np.zeros(len_group_B)])

        if connection_constraint_list:
            P = lg.block_diag(P,200*np.eye(len_conn))
            q = np.block([q,-2*buffer_connect*np.ones(len_conn)])
        
        # inter avoidance constraints
        len_inter_B = len(inter_cons_B)
        G_1 = np.zeros((len_inter_B,l))
        G_1[0:len_inter_B,0:len_U] = -inter_cons_A @  Phi @ VB
        G_1[0:len_inter_B,len_U:len_U+len_E] = inter_cons_C

        h_1 =  inter_cons_A @  Phi @ ( VA @ state + VC) - inter_cons_B 

        # E lower bound constraint
        G_2 = np.zeros((len_E,l))
        G_2[0:len_E,len_U:len_U+len_E] = -np.eye(len_E)
        
        h_2 = np.zeros(len_E)

        # E upper bound constraint
        G_3 = np.zeros((len_E,l))
        G_3[0:len_E,len_U:len_U+len_E] = np.eye(len_E)
        
        h_3 = np.ones(len_E)*epsilon

        # obstacle aviodance constriants

        G_4 = np.zeros((len_ob_B,l))
        G_4[0:len_ob_B,0:len_U] = - ob_cons_A @  Phi @ VB
        G_4[0:len_ob_B,len_U+len_E:len_U+len_E+len_ob_B] = -np.eye(len_ob_B)

        h_4 =  ob_cons_A @  Phi @ ( VA @ state + VC) - ob_cons_B -buffer

        # ob_B upper bound constraints

        G_5 = np.zeros((len_ob_B,l))
        G_5[0:len_ob_B,len_U+len_E:len_U+len_E+len_ob_B] = np.eye(len_ob_B)

        h_5 = np.ones(len_ob_B)*buffer

        # group collision aviodance constraints

        G_6=np.zeros((len_group_B,l))
        G_6[0:len_group_B,0:len_U] = - group_cons_A @  Phi @ VB
        G_6[0:len_group_B,len_U+len_E+len_ob_B:len_U+len_E+len_ob_B+len_group_B] = -np.eye(len_group_B)

        h_6 = group_cons_A @  Phi @ ( VA @ state + VC) - group_cons_B -buffer 


        # group_B up bound constraints

        G_7 = np.zeros((len_group_B,l))
        G_7[0:len_group_B,len_U+len_E+len_ob_B:len_U+len_E+len_ob_B+len_group_B] = np.eye(len_group_B)

        h_7 = np.ones(len_group_B)*buffer


        # linear constraints summary

        l_nonnegative_orthant = len(h_1) + len(h_2) + len(h_3) + len(h_4) + len(h_5) + len(h_6) +len(h_7)
        

        # connection constraints

        if connection_constraint_list:

            G_8 = np.zeros((len_conn,l))
            G_8[0:len_conn,len_U+len_E+len_ob_B+len_group_B:len_U+len_E+len_ob_B+len_group_B+len_conn] = np.eye(len_conn)

            h_8 = np.ones(len_conn)*buffer_connect

            G_9 = np.zeros((len_conn,l))
            G_9[0:len_conn,len_U+len_E+len_ob_B+len_group_B:len_U+len_E+len_ob_B+len_group_B+len_conn] = -np.eye(len_conn)

            h_9 = np.zeros(len_conn)

            l_nonnegative_orthant += len(h_8) + len(h_9)



        # terminal constraint

        A = np.zeros((D,l))
        A[0:D,0:len_U] = Xi_K @ VB

        b= np.zeros(D) - Xi_K @ ( VA @ state + VC)


        # acceleration constraints

        G_cone_1 = np.zeros(((D+1)*K,l))
        h_cone_1 = np.zeros((D+1)*K)
        for k in range(K):
            e_k = np.zeros((D,D*K))
            e_k[0:D,D*k:D*k+D] = np.eye(D)
            G_cone_1[(D+1)*k+1:(D+1)*(k+1),0:len_U] = e_k
            h_cone_1[(D+1)*k] = Umax


        # velocity constraints

        G_cone_2 = np.zeros(((D+1)*K,l))
        h_cone_2 = np.zeros((D+1)*K)
        for k in range(K):
            e_k = np.zeros((D,D*K))
            e_k[0:D,D*k:D*k+D] = np.eye(D)
            G_cone_2[(D+1)*k+1:(D+1)*(k+1),0:len_U] = e_k @ Xi @ VB 
            h_cone_2[(D+1)*k] = Vmax
            h_cone_2[(D+1)*k+1:(D+1)*(k+1)] = - e_k @ Xi @ (VA @ state + VC)

        if connection_constraint_list:
            len_conn_list=len(connection_constraint_list)
            G_cone_3 = np.zeros( (len_conn_list*K*(D+1),l) )
            h_cone_3 = np.zeros(len_conn_list*K*(D+1))
            for i in range(len_conn_list):
                for k in range(K):
                    center=connection_constraint_list[i][0][k]
                    r=connection_constraint_list[i][1][k]
                    e_k = np.zeros((D,D*K))
                    e_k[0:D,D*k:D*k+D] = np.eye(D)
                    G_cone_3[(i*K+k)*(D+1),len_U+len_E+len_ob_B+len_group_B:len_U+len_E+len_ob_B+len_group_B+len_conn]=1.0
                    G_cone_3[(i*K+k)*(D+1)+1:(i*K+k+1)*(D+1),0:len_U]=e_k @ Phi @ VB
                    h_cone_3[(i*K+k)*(D+1)] = r 
                    h_cone_3[(i*K+k)*(D+1)+1:(i*K+k+1)*(D+1)] = - e_k @ Phi @ (VA @ state + VC) + center

        

        if connection_constraint_list:
            G=np.block([ [G_1],[G_2],[G_3],[G_4],[G_5],[G_6],[G_7],[G_8],[G_9],[G_cone_1],[G_cone_2],[G_cone_3] ])
            h=np.block([h_1,h_2,h_3,h_4,h_5,h_6,h_7,h_8,h_9,h_cone_1,h_cone_2,h_cone_3]) 
            dims = {
                    'l': l_nonnegative_orthant, 
                    'q': [D+1 for i in range(K+K)]+[D+1 for i in range(len_conn_list*K)], 
                    's': []
                    }
        else:
            G=np.block([ [G_1],[G_2],[G_3],[G_4],[G_5],[G_6],[G_7],[G_cone_1],[G_cone_2] ])
            h=np.block([h_1,h_2,h_3,h_4,h_5,h_6,h_7,h_cone_1,h_cone_2]) 
            dims = {
                    'l': l_nonnegative_orthant, 
                    'q': [D+1 for i in range(K+K)], 
                    's': []
                    }
            

        G = matrix(G)
        h = matrix(h)
        A = matrix(A)
        b = matrix(b)
        P=2*matrix(P)
        q=matrix(q)


        options.update({'show_progress':False}) 
        

        res = solvers.coneqp(P=P,q=q,G=G,h=h,A=A,b=b,dims=dims)

        U = (np.array(res['x'][0:len_U]))[:,0]
        E = (np.array(res['x'][len_U:l]))[:,0]
        
        self.cache= [VA @ state + VB @ U + VC, U.reshape((K,D)), E[1:inter_cons_C.shape[1]]] 



    def post_processing(self,interval=1):

        # data collection
        U_list=self.cache[1]
        
        
        # get new input
        self.u=U_list[0]

        # get predeterminted trajectory and the terminal point
        P=self.Phi @ self.cache[0]
        P=P.reshape(( self.K, self.D))
        self.terminal_p=P[-1].copy()

        self.data=np.block([[self.data],[self.tractive_point],[self.p],[self.v],\
        [-7777777*np.ones(self.D)],[P],[-9999999*np.ones(self.D)]])

        # here, in our code, PT including the current position in next time's replanning  
        self.pre_traj=np.block([[P],[self.terminal_p]])        
        
        

        if not self.REALFLY:
            # 在仿真状态中认为生成轨迹中的第一个位置就是下一时刻的位置
            # get new state
            self.p[0:self.D]=self.cache[0][0:self.D].copy()
            self.v[0:self.D]=self.cache[0][self.D:2*self.D].copy()
            self.state=np.append(self.p,self.v)

            # get position list
            self.position=np.block([ [self.position],[self.p] ])
        else:
            ####### the realfly #######

            # get the input trajectory
            self.input_traj=self.get_input_traj()

            # 在真实运行过程中，会认为本次生成轨迹之后再过 (interval-1)*h 时间后才是下次计算时初始状态的时刻
            self.p[0:self.D]=self.cache[0][2*self.D*(interval-1)+0:2*self.D*(interval-1)+self.D].copy()
            self.v[0:self.D]=self.cache[0][2*self.D*(interval-1)+self.D:2*self.D*(interval-1)+2*self.D].copy()
            self.state=np.append(self.p,self.v)

            # get new state and predetermined tarjectory for crazyfile
            self.get_pre_traj(interval)

            # get position list
            for i in range(1,interval+1):
                p=self.cache[0][2*self.D*(i-1)+0:2*self.D*(i-1)+self.D].copy()
                self.position=np.block([ [self.position],[p] ])

        # get predtermined trajectory list of each time step
        self.pre_traj_list+=[self.pre_traj]

        # get new cost_index
        for i in range( self.K,-1,-1):
            if( np.linalg.norm( self.pre_traj[i]-self.target ) > 0.01 ):
                break
        self.cost_index=i

        self.W=0.5*np.block([self.epsilon,self.E])+0.5*self.W

        for i in range(len(self.W)):
            if self.W[i] < 0.005:
                self.W[i]=0.005

        # get terminal overlap
        self.get_term_overlap()
        
        
        return None


    # transform the predetermined trajectory to the one that can be parsed by crazyfile
    def get_input_traj(self):    

        P=self.pre_traj

        P=np.block([[self.state[0:2]],[P],[P[-1]]]) 
        self.or_pre_traj=P.copy() 

        t = self.h*np.arange(len(P))
        W=np.ones(len(P))
        polyx = np.polyfit(t, P[:,0], 7,w=W)  
        polyy = np.polyfit(t, P[:,1], 7,w=W)

        polyz = np.zeros(8)
        polyz[7]= self.height
        polyyaw = np.zeros(8)
        polyyaw[7] = self.yaw

        t=self.h*(len(P)-1)

        # 这个地方进行了进一部分进行了进一步的修改
        input_traj = [polyx, polyy, polyz, polyyaw]

        return input_traj

        
    # 在crazyfile情况下获得下时刻的状态估计，并且获得估计的预设轨迹，值得一提的是这里的interval是下次计算所需要的时间
    def get_pre_traj(self,interval):

        l=len(self.pre_traj)

        self.pre_traj[0:l-(interval-1)]=self.pre_traj[interval-1:l].copy()

        for i in range(interval-1):
            self.pre_traj[l-1-i]=self.terminal_p.copy() 

        return  None


    # judging wether the terminal overlap happens or not 
    def get_term_overlap(self):

        term_p=self.pre_traj[-1].copy()
         
        term_second_p=self.pre_traj[-2].copy()
        # term_thrid_p=self.pre_traj[-3].copy()

        self.E=self.cache[2]


        if self.term_overlap:
            
            condition_a=np.linalg.norm(term_p-self.term_last_p)<0.001
            condition_b=np.linalg.norm(term_p-term_second_p)<0.005
            condition_c=np.linalg.norm(term_p-self.target)>0.02

            if condition_a and condition_b and condition_c:
                self.term_overlap_again=True

        else:

            condition_a=np.linalg.norm(term_p-self.term_last_p)<0.015
            condition_b=np.linalg.norm(term_p-term_second_p)<0.02
            condition_c=np.linalg.norm(term_p-self.target)>0.02

            if condition_a and condition_b and condition_c:
                self.term_overlap=True

                # print(str(self.index)+" begins terminal overlap mode")
        
        flag=False

        if(type(self.E) is np.ndarray):
            if (self.epsilon-self.E < 1e-3).all():
                flag=True
        elif self.epsilon-self.E < 1e-3:
                flag=True

        if flag:
            self.term_overlap=False
            self.term_overlap_again=False
            self.eta=1.0


        self.term_index+=1

        if self.term_overlap_again and self.eta < 4.0: # and self.term_index > 3:
            self.term_overlap_again=False
            self.eta += 0.3
            self.term_index = 0
           
        self.term_last_p=term_p.copy()

        # print(str(self.index)+"'s terminal overlap is: "+str(self.term_overlap)+" and "+str(self.E))
        
        return None
        

    # get the list of tractive point which is used for tracting the agent to the tractive point 
    def get_tractive_point_list(self):

        G_p=self.tractive_point
        for i in range(1,self.K):
            G_p=np.append(G_p,self.tractive_point)
        self.G_p=G_p

        return None


    # get the tractive point 
    def get_tractive_point(self):


        if self.type == "Searcher":

            obstacle_list=self.obstacle_list
            
            # if the path is None, i.e, the search based planning doesn't a feasible path, the tractive point will be chosen as the terminal point of predetermined trajectory
            if self.path is None:
            
                self.tractive_point=self.terminal_p.copy() # 这个地方以后可能还需要一定的修改
            
            else:

                for i in range(len(self.path)):

                    if np.linalg.norm(self.path[i] - self.tractive_point) < 1e-2:

                        p_start = i
                
                
                for i in range(len(self.path)-1,p_start-1,-1):

                    vertex_list = [self.terminal_p]

                    vertex_list += [self.path[k] for k in range(i,p_start-1,-1)]

                    # print(vertex_list)

                    poly = polygon(vertex_list)

                    if not detect_polygon_collision(obstacle_list,poly):

                        self.tractive_point = self.path[i].copy()

                        break
            
                if detect_line_collision(obstacle_list,line(self.terminal_p,self.tractive_point)):

                    for i in range(len(self.path)):

                        if np.linalg.norm(self.path[i] - self.tractive_point) < 1e-2:

                            p_start = i
                        
                    # if a collision-free path exists, then we can find the tractive point
                    for i in range(p_start,-1,-1):
                        
                        if not detect_line_collision(obstacle_list,line(self.path[i],self.terminal_p)):
                            
                            self.tractive_point = self.path[i].copy()
                            
                            break
                    
        else:

            self.tractive_point = self.terminal_p.copy()

        # No matter whether path exists or doesn't, there needs a tractive list for convex programming    
        self.get_tractive_point_list()
        
        return None



    def get_nei_objective(self,share_data):

        pre_traj_list=share_data['pre_traj']

        # P_neighbor includes all horizon's position
        P_neighbor=np.zeros(self.K*self.D)

        den=0
        
        for nei in self.neighbor:
            if nei < self.index:
                P_neighbor+=pre_traj_list[nei][1:self.K+1].reshape(1,-1)[0]
                den+=1
            elif nei>self.index:
                P_neighbor+=2*pre_traj_list[nei][1:self.K+1].reshape(1,-1)[0]
                den+=2

        P_neighbor/=den

        return P_neighbor


#######################################################
#                                                     #
#                                                     #
#######################################################
#                                                     #
#                                                     #
#######################################################


    def get_coef_matrix(self):
        
        D=self.D
        K=self.K

        # position matrix
        # get all position matrix
        global Phi
        Phi=np.column_stack( (np.eye(D),np.zeros((D,D))) )
        phi=Phi
        for i in range(1,K):
            Phi=lg.block_diag(Phi,phi)
        self.Phi=Phi


        # get K position matrix
        global Phi_K
        Phi_K=np.zeros((D,K*D))
        for i in range(0,D):
            Phi_K[i][K*D-D+i]=1.0
        self.Phi_K=Phi_K @ Phi

        # velocity matrix
        global Xi
        Xi=np.column_stack( (np.zeros((D,D)),np.eye(D)) )
        xi=Xi
        for i in range(1,K):
            Xi=lg.block_diag(Xi,xi)
        self.Xi=Xi

        # get K velocity matrix
        global Xi_K
        Xi_K=np.zeros((D,K*D))
        for i in range(0,D):
            Xi_K[i][K*D-D+i]=1.0
        self.Xi_K=Xi_K @ Xi
        
        # gamma this matrix is used for the maximium input control constraint 
        theta_u=np.array([1.0,1.0])
        Theta_u=theta_u
        for i in range(1,K):
            Theta_u=lg.block_diag(Theta_u,theta_u)
        self.Theta_u=Theta_u

        self.Theta_v=Theta_u.copy()

        self.Theta_p=Theta_u.copy()


        
        
        # control input change cost
        
        Delta=np.eye(K*D)
        for i in range(D):
            Delta[i][i]=0
        for i in range(D,K*D):
            Delta[i][i-D]=-1

        self.Delta=Delta.T @ Delta

        
        Delta_P=np.zeros((K*D,K*D))
        for i in range(1,K):
            for j in range(D):
                Delta_P[i*D+j][i*D+j]=i/K
                Delta_P[i*D+j][i*D-D+j]=-i/K
        
        self.Delta_P=Delta_P.T @ Delta_P

        return None


    def get_dynamic(self):

        K=self.K
        h=self.h
        D=self.D

        # system dynamic in continous time
        A=np.zeros((2*D,2*D))
        B=np.zeros((2*D,D))
        A[0:D,D:2*D]=np.eye(D)
        B[D:2*D,0:D]=np.eye(D)

        m=A.shape[0]

        # system dynamic
        A=np.dot(np.linalg.inv(np.eye(m)-h/2*A),(np.eye(m)+h/2*A))
        B=np.dot(np.linalg.inv(np.eye(m)-h/2*A)*h,B)

        VA=A
        for i in range(2,K+1):
            C=np.eye(m)
            for j in range(1,i+1):
                C=np.dot(C,A)
            VA=np.block([[VA],[C]])
        self.VA=VA

        VB=B
        for i in range(1,K):
                VB=np.block( [ [ np.dot( np.zeros((m,m)),B ) ],[VB] ] )
        for i in range(1,K):
            C=np.dot( matrixPow(A,i-K+1),B )
            for j in range(i-K+2,i+1):
                C=np.block([[C],[np.dot(matrixPow(A,j),B)]])
            VB=np.block([[C,VB]])
        self.VB=VB

        self.VC=np.zeros(m*K)

        return None


# the power of matrix
def matrixPow(Matrix,n):
    if(type(Matrix)==list):
        Matrix=np.array(Matrix)
    if(n==1):
        return Matrix
    elif(n==0):
        return np.eye(Matrix.shape[0])
    elif(n<0):
        return np.zeros(Matrix.shape)
    else:
        return np.matmul(Matrix,matrixPow(Matrix,n-1))