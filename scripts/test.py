################################################################
# 
# Author: Mike Chen 
# From Peking university
# Last update: 2023.4.24
# 
################################################################


import      SET
SET.initial_set()
from        run         import run_one_step
from        others      import get_shared_data,distance,save_data,check_reach_target
from        agent.uav   import uav
from plot.plot_mayavi import plot_position,plot_pre_traj
import      numpy       as     np
import sys
sys.path.append("..")
import time
import shutil
import os
from trajectory import land
from decision import OptTree


def initialize():

    # inilization 
    agent_list=[]
    for i in range(SET.Num):
        agent_list+=[ uav(
            index=i,
            type=SET.type_list[i],
            K=SET.K,
            h=SET.h,
            D=SET.D,
            ini_p=SET.ini_p[i],
            Num=SET.real_num,
            REALFLY=False,
            obstacle_list=SET.obstacle_list,
            path=SET.path_list[i],
            buffer=SET.buffer,
            buffer_connect=SET.buffer_connect,
            neighbour=SET.neighbour_list[i]
        ) ]


    return agent_list


def main():

    # buld the file that saving the data and figure
    if os.path.exists('savefig'):
        shutil.rmtree('savefig')
    
    os.mkdir('savefig')


    OptTree(SET.p_anchor,SET.target_list)


    # initialize the agnets
    agent_list=initialize()


    all_time=0.0

    # begin the main loop
    for epi in range(1,SET.episodes+1):
        
        print('==============================================')
        
        start=time.time()
       
        # run one step 
        agent_list = run_one_step(
                            agent_list,
                            obstacle_list=SET.obstacle_list,
                            ini_obstacle_list=SET.ini_obstacle_list,
                            group_list=SET.group_list,
                            connection_list=SET.connection_list,
                            d_connect=SET.d_connect,
                            compute_model=SET.compute_model,
                            core_num=SET.core_num
                        )
        
        time_interval=time.time()-start
        
        # print running time in this step
        print('Step '+str(epi)+' has finished, running time is: '+str(round(time_interval,4)))
        print(' ')
        all_time+=time_interval
        
        # plot the predetermined tarjectories in this time step
        if epi > 80:
            plot_pre_traj(
                agent_list=agent_list,
                obstacle_list=SET.ini_obstacle_list,
                plot_range=SET.plot_range,
                episodes=epi,
                format=SET.format,
                show=SET.show,
                connect_list=SET.connection_list,
                target_list=SET.target_list
            )
    
        # juding whether all robots reach to their target positions
        if check_reach_target(agent_list):
            break 

    # plot trajectories in this test
    plot_position(
        agent_list=agent_list,
        obstacle_list=SET.ini_obstacle_list,
        plot_range=SET.plot_range,
        format=SET.format,
        connect_list=SET.connection_list,
        target_list=SET.target_list
    )

    # save the running data
    save_data(agent_list)

    print('===================================================')
    print('The whole running time is: '+str(round(epi*SET.h,2))+'s')
    print('The total distance of all the agent is: '+str(round(distance(agent_list),2))+'m')
    print('The mean time per replanning is: '+str(round(1000*all_time/epi,4))+'ms')
    print('Th mean time per replanning per agent is:'+str(1000*round(all_time/epi/SET.Num,4))+'ms' )
    print('The data has been saved in: data/data.csv')
    print('===================================================')


if __name__ == '__main__':

    main()