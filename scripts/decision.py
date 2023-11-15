import numpy as np
import copy
import SET
import multiprocessing as mp
from plot.plot_mayavi import plot_path_tree


def run(item):
    planner=item[0]
    current_path=item[1]
    current_num_list=item[2]
    return planner.rrt_star_planning(goal_list=current_path, goal_num_list=current_num_list)


def OptTree(p_anchor,target_list):

    
    still_list = [i for i in range(len(target_list))]
    path_tree = []

    obstacleList=SET.obstacle_list


    from RRT import RRT

    randArea = [
        [SET.map_range['x'][0],SET.map_range['x'][1]],
        [SET.map_range['y'][0],SET.map_range['y'][1]],
        [SET.map_range['z'][0],SET.map_range['z'][1]]
    ]

    planner_list=[RRT(randArea=randArea, start=target, r_c=SET.d_connect,obstacleList=obstacleList, maxIter=1000) for target in target_list]

    # Set params
    current_path = [p_anchor]
    current_num_list = [0]

    import time

    start_time=time.time()
    for _ in planner_list:

        print('======  adding new branch  =====')

        items=[]

        for i in still_list:
            item=[planner_list[i],current_path,current_num_list]
            items+=[item]

        pool = mp.Pool(len(still_list))
        new_planner_list = pool.map(run, items)
        pool.close()
        pool.join()

        for i in range(len(still_list)):
            planner_list[still_list[i]]=new_planner_list[i]

        path_list = [planner.min_path for planner in new_planner_list]
        
        root_num_list = [planner.root_num for planner in new_planner_list]

        eva_list = [len(path)+root_num for path,root_num in zip(path_list,root_num_list)]

        MinInd = eva_list.index(min(eva_list))

        current_path = path_list[MinInd]

        path_tree+=[copy.deepcopy(current_path)]

        np.delete(current_path,0) 

        current_num_list = [i+planner_list[MinInd].root_num for i in range(1,len(current_path)+1)]

        still_list.remove(still_list[MinInd])

    
    print('Total time cost is '+str(time.time()-start_time))

    
    plot_path_tree(
        path_tree,
        obstacle_list=SET.ini_obstacle_list,
        target = SET.target_list,
        format=SET.format
    )

    SET.Num = 1
    SET.type_list=["Anchor"]
    SET.group_list = []
    SET.path_list = [None]

    position_list = [SET.p_anchor]
    parent_list = [None]

    for branch in path_tree:
        for i in range(1,len(branch)):
            position_list+=[branch[i]]

            SET.type_list+=["Connector"]

            for target in SET.target_list:
               if np.linalg.norm(branch[i]-target) < 1e-4:  
                    SET.type_list[-1]="Searcher"
                    break
                

            for j in range(len(position_list)):
                if np.linalg.norm(position_list[j]-branch[i-1])<1e-4:
                    parent_list+=[j]
                    break
    

    SET.real_num = 0
    for type in SET.type_list:
        if type!="Anchor":
            SET.real_num+=1

    SET.Num=len(SET.type_list)

    for i in range(1,len(parent_list)):
        SET.group_list+=[[parent_list[i],i]]
        if SET.type_list[i]=="Searcher":
            SET.path_list+=[get_path(parent_list=parent_list,position_list=position_list,Ind=i)]
        else:
            SET.path_list+=[None]
    

    SET.connection_list = copy.deepcopy(SET.group_list)

    # print(SET.connection_list)
    # print(parent_list)
    # print(SET.type_list)
    # print(position_list)
    # print(SET.path_list)

    neighbour_list = []
    for i in range(SET.Num):
        neighbour=[]
        for j in range(len(SET.connection_list)):
            if i in SET.connection_list[j]:
                nei= SET.connection_list[j][1-SET.connection_list[j].index(i)]
                if nei not in neighbour:
                    neighbour+=[nei]

        neighbour_list+=[neighbour]
    SET.neighbour_list = neighbour_list

    SET.ini_p=[SET.p_anchor]

    l = np.ceil(np.sqrt(SET.Num)) 

    for i in range(1,SET.Num):
        SET.ini_p += [SET.p_anchor+np.array([10*(i%l),10*int(i/l),SET.p_anchor[2]])]

    # print(SET.group_list)
    # print(SET.type_list)
    # for path in SET.path_list:
    #     print(path)

    return None


def get_path(position_list,parent_list,Ind):
    
    path=[position_list[Ind]]
    while parent_list[Ind] is not None:
        Ind=parent_list[Ind]
        path+=[position_list[Ind]]

    path.reverse()

    return np.array(path)



def path_length(path):

    length = 0
    for i in range(1,len(path)):
        length += np.linalg.norm(path[i] - path[i-1])

    return length 