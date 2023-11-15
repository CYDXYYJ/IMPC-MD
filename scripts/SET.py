import numpy as np

def initial_set():

    ###################
    #  personal set   #
    ###################

    global Num     # the number of agents
    Num = 0

    # Type:
    #       Searcher
    #       Connector 
    #       Anchor: ground station
    global type_list
    type_list=[]

    global real_num
    real_num = 0
    global D       # deminision
    D=3
    global K       # the length of horizon
    K=11
    global h       # time step
    h=0.5
    global episodes      # the maximum times for replanning
    episodes=10
    global compute_model # the mode that running the convex programming, 'norm' , 'process' and 'thread'
    compute_model='process'      
    global core_num      # if the mode is 'process', then, choosing the number of core apllied for this computing
    core_num=3
    global map_range
    map_range={'x':[0.0,500.0],'y':[0.0,500.0],'z':[-10.0,95]}

    ## about the fig ##
    global show          # if show = True, then the fig will be save in file: savefig. 
    show=True            # Pay attentiion that, in realfly test, show must be chosen as 'False' since it will lead to the time delay


    global format # the format of the printed fig
    format='.jpg'


    global plot_range    # if save figure, determine the plot range of this figure
    plot_range={'x':[map_range['x'][0]-0.5,map_range['x'][1]+0.5],'y':[map_range['y'][0]-0.5,map_range['y'][1]+0.5],'z':[map_range['z'][0]-0.5,map_range['z'][1]+0.5]}
    plot_range['size']=(10,10)



    global buffer        # the buffer when adopting parting plane linear constraints
    buffer = 6.0         # "obstacle-transitor":0.02

    global d_connect     # the maximum connection distance 
    d_connect = 150.0

    global buffer_connect # The buffer width for connection
    buffer_connect = 6.0

    global radius   # The extended width when considering the radius of agents, pay attention: it is radius
    radius = 2.0

    # the position of the ground station
    global p_anchor
    p_anchor = np.array([10.0,10.0,5.0])

    global ini_p   # intial position
    ini_p = []

    global path_list 
    path_list = []


    # target position: is a variable in some condition
    global target_list
    target_list=[
        np.array([450.0,370.0,50.0]),
        np.array([200.0,100.0,50.0]),
        np.array([400.0,150.0,50.0]),
        np.array([320.0,250.0,50.0]),
        np.array([150.0,225.0,50.0]),
        np.array([265.0,325.0,50.0]),
        np.array([230.0,450.0,50.0])
        # np.array([400.0,20.0,50.0]),
        # np.array([180.0,330.0,50.0]),
        # np.array([50.0,400.0,50.0])
    ]


    global group_list
    group_list=[]


    global connection_list
    connection_list=[]


    global neighbour_list
    neighbour_list=[]
    

    from geometry.geometry3d import box,polygon


    # the enviromrnt doen't consider the diameter ofrobots
    global ini_obstacle_list 
    ini_obstacle_list=[
        polygon([

            np.array([-50.0,550.0,-5.0]),
            np.array([-50.0,-50.0,0.-5.0]),
            np.array([550.0,-50.0,-5.0]),
            np.array([550.0,550.0,-5.0])
        ]),

        # 1
        polygon([
            np.array([ 50.0, 75.0,0.0]),
            np.array([135.0, 65.0,0.0]),
            np.array([150.0,150.0,0.0]),
            np.array([ 65.0,175.0,0.0]),
            np.array([ 50.0,75.0,100.0]),
            np.array([135.0, 65.0,100.0]),
            np.array([150.0,150.0,100.0]),
            np.array([ 65.0,175.0,100.0])
        ]),

        # 3
        polygon([
            np.array([50.0,225.0,0.0]),
            np.array([105.0,215.0,0.0]),
            np.array([120.0,250.0,0.0]),
            np.array([ 65.0,275.0,0.0]),
            np.array([50.0,225.0,100.0]),
            np.array([105.0,215.0,100.0]),
            np.array([120.0,250.0,100.0]),
            np.array([ 65.0,275.0,100.0])
        ]),

        # 2
        polygon([
            np.array([275.0,45.0,0.0]),
            np.array([405.0,40.0,0.0]),
            np.array([420.0,125.0,0.0]),
            np.array([315.0,135.0,0.0]),
            np.array([275.0,45.0,100.0]),
            np.array([405.0,40.0,100.0]),
            np.array([420.0,125.0,100.0]),
            np.array([315.0,135.0,100.0])
        ]),

        # 4
        polygon([
            np.array([175,200.0,0.0]),
            np.array([280,190.0,0.0]),
            np.array([295.0,275.0,0.0]),
            np.array([185.0,300.0,0.0]),
            np.array([175,200.0,100.0]),
            np.array([280,190.0,100.0]),
            np.array([295.0,275.0,100.0]),
            np.array([185.0,300.0,100.0])
        ]),

        polygon([
            np.array([75,350.0,0.0]),
            np.array([155,340.0,0.0]),
            np.array([170.0,425.0,0.0]),
            np.array([90.0,420.0,0.0]),
            np.array([75,350.0,100.0]),
            np.array([155,340.0,100.0]),
            np.array([170.0,425.0,100.0]),
            np.array([90.0,420.0,100.0])
        ]),

        polygon([
            np.array([275,350.0,0.0]),
            np.array([355,340.0,0.0]),
            np.array([370.0,425.0,0.0]),
            np.array([290.0,420.0,0.0]),
            np.array([275,350.0,100.0]),
            np.array([355,340.0,100.0]),
            np.array([370.0,425.0,100.0]),
            np.array([290.0,420.0,100.0])
        ]),

        polygon([
            np.array([350,200.0,0.0]),
            np.array([430,190.0,0.0]),
            np.array([440.0,325.0,0.0]),
            np.array([390.0,330.0,0.0]),
            np.array([350,200.0,100.0]),
            np.array([430,190.0,100.0]),
            np.array([440.0,325.0,100.0]),
            np.array([390.0,330.0,100.0])
        ])
    ]

    # the obstacle-inflated enviroment which consider the diameter of robots
    global obstacle_list
    from copy import deepcopy
    obstacle_list=deepcopy(ini_obstacle_list) 