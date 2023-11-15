################################################################
# 
# Author: Mike Chen 
# From Peking university
# Last update: 2023.8.3
# 
#
################################################################


import copy
import math
import random
import time
import numpy as np
import geometry.opengjk.opengjkc as opengjk



class RRT:

    def __init__(self, obstacleList, start, r_c, randArea, goalSampleRate=150, maxIter=200):

        self.rand_x = randArea[0]
        self.rand_y = randArea[1]
        self.rand_z = randArea[2]

        # 通信距离
        self.r_c=r_c

        # 单次延伸长度
        self.expand_dis = self.r_c/2

        self.goal_sample_rate = goalSampleRate
        self.max_iter = maxIter
        self.obstacle_list = obstacleList
        self.start = Node(start)
        self.node_list = [self.start]
        self.min_path = None


    def rrt_star_planning(self, goal_list, goal_num_list):


        start_time = time.time()
        
        self.node_list = [self.start]
        self.goal_list = [Node(goal) for goal in goal_list]
        self.goal_num=len(self.goal_list)
        path_list = [None for _ in goal_list]
        lastPathLength_list = [float('inf') for _ in goal_list]
        lastNum_list = [float('inf') for _ in goal_list]

        # Computing the sampling space
        cBest_list = [float('inf') for _ in goal_list]
        cMin_list = [np.linalg.norm(self.start.p-goal.p) for goal in self.goal_list]
        xCenter_list = [(self.start.p+goal.p)/2 for goal in self.goal_list]
        a1_list = [(goal.p-self.start.p)/np.linalg.norm(goal.p-self.start.p) for goal in self.goal_list]

        e_theta_list = [math.atan2(a1[1], a1[0]) for a1 in a1_list]
        h_theta_list = [-math.asin(a1[2]) for a1 in a1_list]

        # 直接用二维平面上的公式（2选1）
        C_e_list = [
            np.array([[math.cos(e_theta), -math.sin(e_theta), 0],
                      [math.sin(e_theta), math.cos(e_theta),  0],
                      [0,                 0,                  1]])
            for e_theta in e_theta_list
        ]
        
        C_h_list = [
            np.array([[math.cos(h_theta), 0, math.sin(h_theta)],
                      [0,                 1,                 0],
                      [-math.sin(h_theta),0, math.cos(h_theta)]])
            for h_theta in h_theta_list
        ]

        C_list = [C_e @ C_h for C_e, C_h in zip(C_e_list,C_h_list)]

        # 开始的时候就检查一下能否进行直连 

        for j in range(len(self.goal_list)):
            if self.is_near_goal(self.start,self.goal_list[j]):
                if self.check_collision(self.start, self.goal_list[j]):
                    path = np.array([self.goal_list[j].p,self.start.p])
                    path_list[j] = path
                    lastNum_list[j] = 1
                    lastPathLength_list[j] = self.start.dis(self.goal_list[j])
                    cBest_list[j] = self.start.dis(self.goal_list[j])


        for t in range(self.max_iter):

            # 找采样点
            rnd = self.informed_sample(cBest_list, cMin_list, xCenter_list,C_list)
            # rnd=self.sample(0)

            # 找到最近点
            n_ind = self.get_nearest_list_index(self.node_list, rnd)
            nearestNode = self.node_list[n_ind]

            # 确定搜索方向
            dir=rnd-nearestNode.p
            dir=dir/np.linalg.norm(dir)
            # 获得新点
            newNode = self.get_new_node(dir, n_ind, nearestNode)
            
            # 如果与最近点之间连线是不碰撞
            if self.check_collision(newNode,nearestNode):
                # 寻找新点的邻近点
                nearInds = self.find_near_nodes(newNode)
                # 选择新节点的父节点，规则是总的cost最小
                newNode = self.choose_parent(newNode, nearInds)
                # 添加新点
                self.node_list.append(newNode)
                # 这一步是对树的重构
                self.rewire(newNode, nearInds)

                # 假设离目标点近的话，则判断能否连接到目标点
                for j in range(len(self.goal_list)):
                    if self.is_near_goal(newNode,self.goal_list[j]):
                        lastIndex = len(self.node_list) - 1
                        tempNum = self.node_list[lastIndex].num+1
                        if tempNum > lastNum_list[j]:
                            continue
                        tempPath = self.get_final_course(lastIndex,self.goal_list[j])
                        tempPathLen = self.get_path_len(tempPath)
                        
                        if (lastNum_list[j]>tempNum) or (lastNum_list[j]==tempNum and lastPathLength_list[j] > tempPathLen):
                            if self.check_collision(newNode, self.goal_list[j]):
                                path_list[j] = tempPath
                                lastNum_list[j] = tempNum
                                lastPathLength_list[j] = tempPathLen
                                cBest_list[j] = tempPathLen

                if time.time()-start_time > 0.4:
                    break
                
        # print(t)

        num_list = [len(path) for path in path_list if path is not None]

        min_num = min(num_list)

        Ind_list = [i for i in range(len(path_list)) if len(path_list[i]) == min_num]

        length_list = [path_len(path_list[i]) for i in Ind_list]

        Min_ind = Ind_list[length_list.index(min(length_list))]
        

        if self.min_path is None:
            self.min_path = copy.deepcopy(path_list[Min_ind])
            self.len = min(length_list)
            self.root_num = goal_num_list[Min_ind]
        else:
            if len(self.min_path)>len(path_list[Min_ind]) or (len(self.min_path)==len(path_list[Min_ind]) and min(length_list) <self.len):
                self.min_path = copy.deepcopy(path_list[Min_ind])
                self.len = min(length_list)
            self.root_num = goal_num_list[Min_ind]

        return self


    def sample(self,i):
        rand = random.randint(0, 1000)
        if  rand >= self.goal_sample_rate:
            rnd = np.array(
                [random.uniform(self.rand_x[0], self.rand_x[1]), random.uniform(self.rand_y[0], self.rand_y[1]),random.uniform(self.rand_z[0], self.rand_z[1])]
            )
        else:  # goal point sampling
            rnd = self.goal_list[i].p
        return rnd


    def informed_sample(self, cBest_list, cMin_list, xCenter_list, C_list):

        rand = random.randint(0, 999)
        
        i = int(rand/1000*self.goal_num)
        cBest=cBest_list[i]

        cMin=cMin_list[i]

        if cBest < float('inf') and cBest-cMin > 1e-5:
            xCenter=xCenter_list[i]
            C=C_list[i]
            r = [cBest / 2.0,
                 math.sqrt(cBest ** 2 - cMin ** 2) / 2.0,
                 math.sqrt(cBest ** 2 - cMin ** 2) / 2.0]
            L = np.diag(r)

            for _ in range(20):
                xBall = self.sample_unit_ball()
                rnd = C @ L @ xBall + xCenter
                if rnd[2] < self.rand_z[1] and rnd[2] > self.rand_z[0]:
                    return rnd
        else:
            rnd = self.sample(i)

        return rnd


    def find_near_nodes(self, newNode):

        d_list = [np.sum(np.square(node.p-newNode.p)) for node in self.node_list]
        near_inds = [d_list.index(i) for i in d_list if i <=  self.r_c** 2]
        return near_inds

    @staticmethod
    def sample_unit_ball():
        
        r=math.pow(random.uniform(0,1),1/3)
        angle1=random.uniform(0,1)*2*math.pi
        angle2=math.acos(random.uniform(0,1)*2-1)
        x=r*math.cos(angle1)*math.sin(angle2)
        y=r*math.sin(angle1)*math.sin(angle2)
        z=r*math.cos(angle2)

        return np.array([x,y,z])

    @staticmethod
    def get_path_len(path):
        pathLen = 0
        for i in range(1, len(path)):
            node1_x = path[i][0]
            node1_y = path[i][1]
            node2_x = path[i - 1][0]
            node2_y = path[i - 1][1]
            pathLen += math.sqrt((node1_x - node2_x) ** 2 + (node1_y - node2_y) ** 2)

        return pathLen

    @staticmethod
    def get_nearest_list_index(nodes, rnd):
        dList = [np.sum(np.square(node.p-rnd)) for node in nodes]
        minIndex = dList.index(min(dList))
        return minIndex

    def get_new_node(self, dir, n_ind, nearestNode):
        newNode = copy.deepcopy(nearestNode)
        newNode.p += self.expand_dis * dir
        newNode.cost += self.expand_dis
        newNode.parent = n_ind
        newNode.num+=1
        return newNode

    def is_near_goal(self, node, goal):
        d = np.linalg.norm(node.p-goal.p)
        if d < self.r_c:
            return True
        return False

    def choose_parent(self, newNode, nearInds):

        # if len(nearInds) == 0:
        #     return newNode

        mindcost = float('inf')
        minNum = float('inf')

        for i in nearInds:
            d = newNode.dis(self.node_list[i])
            if self.node_list[i].num + 1 < minNum or (self.node_list[i].num + 1 == minNum and d + self.node_list[i].cost < mindcost):
                if self.check_collision(self.node_list[i], newNode):
                    minInd = i
                    mindcost = d + self.node_list[i].cost
                    minNum = self.node_list[i].num + 1

        newNode.cost = mindcost
        newNode.parent = minInd
        newNode.num = minNum

        return newNode

    # 对树进行重构，注意这里nearInds是指新点附近的点
    def rewire(self, newNode, nearInds):

        new_index = len(self.node_list)

        num = newNode.num + 1
        
        for i in nearInds:

            nearNode = self.node_list[i]

            s_cost = newNode.cost + nearNode.dis(newNode)

            if num < nearNode.num or (num == nearNode.num and nearNode.cost > s_cost):
                if self.check_collision(nearNode, newNode):
                    nearNode.parent = new_index - 1
                    nearNode.cost = s_cost
                    nearNode.num = num


    @staticmethod
    def distance_squared_point_to_segment(v, w, p):
        # Return minimum distance between line segment vw and point p
        if np.array_equal(v, w):
            return (p - v).dot(p - v)  # v == w case
        l2 = (w - v).dot(w - v)  # i.e. |w-v|^2 -  avoid a sqrt
        t = max(0, min(1, (p - v).dot(w - v) / l2))
        projection = v + t * (w - v)  # Projection falls on the segment
        return (p - projection).dot(p - projection)
        

    def check_collision(self, Node1, Node2,tor=5):

        if Node1.p[2] > self.rand_z[1] or Node2.p[2] > self.rand_z[1] or Node1.p[2] < self.rand_z[0] or Node2.p[2] < self.rand_z[0]:
            return False

        for ob in self.obstacle_list:
            v1=ob.vertex_list
            v2=[Node1.p,Node2.p]
            d=opengjk.gjk(v1,v2)
            if abs(d) < tor:
                return False

        return True


    def get_final_course(self, lastIndex, goal):
        path = [goal.p]
        while self.node_list[lastIndex].parent is not None:
            node = self.node_list[lastIndex]
            path.append(node.p)
            lastIndex = node.parent
        path.append(self.start.p)
        return np.array(path)


def path_len(path):
    
    length = 0
    for i in range(1,len(path)):
        length += np.linalg.norm(path[i] - path[i-1])
    return length 

class Node:

    def __init__(self, p):
        self.p=p
        self.cost = 0.0
        self.num = 0
        self.parent = None

    def dis(self,node):
        return np.linalg.norm(self.p-node.p) 