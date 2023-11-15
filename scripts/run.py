import SET 
if SET.compute_model=='thread':
    from thread import MyThread
elif SET.compute_model=='process':
    import multiprocessing as   mp

from safe_zone import Get_list_of_group_corridor,Get_group_cons
from distance_constraints import Get_list_of_connection_constraint_list
from inter_avoidance import Get_inter_cons
from others import get_shared_data
import time
import copy



# this is the main loop
def run_one_step(
        agent_list,
        obstacle_list=[],
        ini_obstacle_list=[],
        interval=1,
        next_interval=1,
        group_list=[],
        connection_list=[],
        compute_model='norm',
        d_connect=2.0,
        core_num=1
    ):


    start=time.time()


    ######## communicate data ########

    share_data = get_shared_data(agent_list)


    ######## the main caculation #######
    start=time.time()
    # get group constraints
    list_of_group_corridor_list=Get_list_of_group_corridor(group_list,agent_list,obstacle_list)
    print('Computing connectivity constraints uses '+str(time.time()-start))
    
    # get connection constraints
    list_of_connection_constraint_list=Get_list_of_connection_constraint_list(connection_list,agent_list,d_connect)

    # the calculation for each agent

    items=[]
    for agent in agent_list:
        i=agent.index
        items.append( [ copy.deepcopy(agent),\
                        copy.deepcopy(share_data),\
                        copy.deepcopy(obstacle_list),\
                        copy.deepcopy(list_of_group_corridor_list[i]),\
                        copy.deepcopy(list_of_connection_constraint_list[i]),\
                        next_interval] )

    if compute_model=='thread':

        pool = []
        for agent in agent_list:
            i=agent.index
            thread = MyThread(run_one_agent,args=[items[i]])
            pool.append(thread)
            thread.start()


        agent_list=[]
        for thread in pool:
            thread.join() 
            agent_list.append(thread.get_result())

    elif compute_model=='norm':

        agent_list=[run_one_agent(item) for item in items]
        

    elif compute_model=='process':

        # mp.set_start_method("forkserver")
        # mp.set_start_method("fork")
        start=time.time()
        pool = mp.Pool(core_num)
        agent_list=pool.map(run_one_agent, items)
        pool.close()
        pool.join()
        print('Trajectory planning time is:'+str(time.time()-start))

    else:

        raise ValueError('Please choose a compute model')


    return agent_list

    

def run_one_agent(item):

    agent=item[0]
    share_data=item[1]
    obstacle_list=item[2]
    group_corridor_list=item[3]
    connection_constraint_list=item[4]
    next_interval=item[5]

    agent.obstacle_list=obstacle_list

    # 如果是锚点直接不进行计算
    if agent.type == "Anchor":
        return agent

    # get objective
    if agent.type=="Connector":
        agent.P_neighbor = agent.get_nei_objective(share_data)

    # get connection constraints
    agent.connection_constraint_list=connection_constraint_list

    # get inter robot avoidance constraints
    agent.inter_cons_A,agent.inter_cons_B,agent.inter_cons_C,agent.Rho_ij = Get_inter_cons(agent,share_data)
    
    agent.get_tractive_point()
    

    # get group collision avoidance constraints
    agent.group_cons_A, agent.group_cons_B = Get_group_cons(agent,group_corridor_list)

    # running convex program
    agent.run_convex_program()

    if not agent.REALFLY:
        # post processing
        agent.post_processing()
    else:
        interval=next_interval
        
        agent.post_processing(interval)

    return agent