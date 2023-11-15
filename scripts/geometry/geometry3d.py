import numpy as np
import time
import copy
import multiprocessing as mp
import geometry.opengjk.opengjkc as opengjk


D=3

# definition of a line
class line():
    
    def __init__(self,p1,p2):

        if len(p1) !=3 or len(p2) !=3:
            print("this is a 3d line")

        self.type='line'
        self.p1=p1
        self.p2=p2 
        self.num=2
        self.vertex_list=[p1,p2]


    def is_out_of_plane(self,plane):

        a=plane[0:D]
        b=plane[D]

        f1 = a @ self.p1 + b < 1e-8
        f2 = a @ self.p2 + b < 1e-8

        if f1 and f2:
            return True
        else:
            return False

# definition of a polyhedron
class polygon():

    def __init__(self,vertex_list):

        self.type='polygon'
        self.vertex_list=vertex_list
        self.num=len(vertex_list)
        a=np.zeros(3)
        for i in range(self.num):
            a+=vertex_list[i]
        self.center=a/self.num
        
    
    def is_out_of_plane(self,plane):

        a=plane[0:3]
        b=plane[3]

        for v in self.vertex_list:
            if a @ v + b > 0:
                return False 
        
        return True
    
    def get_minimum_distance(self,p):
        # the get minimum distance to a point

        vertex_list=np.array(self.vertex_list)

        return opengjk.gjk(vertex_list,p)

        

class box():

    def __init__(self,x1,x2,y1,y2,z1,z2):

        self.x1=x1
        self.x2=x2 
        self.y1=y1 
        self.y2=y2 
        self.z1=z1
        self.z2=z2

        self.type='box'
        vertex_list=[np.array([x1,y1,z1]),
                     np.array([x1,y1,z2]),
                     np.array([x1,y2,z1]),
                     np.array([x1,y2,z2]),
                     np.array([x2,y1,z1]),
                     np.array([x2,y1,z2]),
                     np.array([x2,y2,z1]),
                     np.array([x2,y2,z2])]

        self.vertex_list=vertex_list
        self.num=len(vertex_list)
        a=np.zeros(D)
        for i in range(self.num):
            a+=vertex_list[i]
        self.center=a/self.num
    
    def is_out_of_plane(self,plane):

        a=plane[0:D]
        b=plane[D]

        for v in self.vertex_list:
            if a @ v + b > 0:
                return False 
        
        return True

    def get_minimum_distance(self,p):
        # the get minimum distance to a point

        # 先判断在不在其中
        if detect_point_in_ob(self,p):
            return 0.0

        min_p=np.zeros(3)

        if p[0]<self.x1:
            min_p[0]=self.x1
        elif p[0]>self.x2:
            min_p[0]=self.x2
        else:
            min_p[0]=p[0]

        if p[1]<self.y1:
            min_p[1]=self.y1
        elif p[1]>self.y2:
            min_p[1]=self.y2
        else:
            min_p[1]=p[1]

        if p[2]<self.z1:
            min_p[2]=self.z1
        elif p[2]>self.z2:
            min_p[2]=self.z2
        else:
            min_p[2]=p[2]


        return  np.linalg.norm(min_p-p)

# define a box based on its diagonal line formed by p1 and p2
def box_line(p1,p2):

    x1=min(p1[0],p2[0])
    x2=max(p1[0],p2[0])

    y1=min(p1[1],p2[1])
    y2=max(p1[1],p2[1])

    z1=min(p1[2],p2[2])
    z2=max(p1[2],p2[2])

    a=box(x1,x2,y1,y2,z1,z2)
    a.type='box_line'

    return a



# detect whether a point is included in a convex obstacle ob
def detect_point_in_ob(ob, p):

    vertex_list=np.array(ob.vertex_list)

    d=opengjk.gjk(vertex_list,p)

    if abs(d) < 1e-6:
        return True
    else:
        return False


        

# detect whether a point is included in any obstacles
def detect_point_collision(obstacle_list,p):

    for ob in obstacle_list: 
        if detect_point_in_ob(ob,p):
            return True
    return False


# 检测线与障碍物之间的是否有碰撞
def detect_polygon_line_collision(ob,line,tor):

    vertex_list1=np.array(ob.vertex_list)
    vertex_list2=np.array(line.vertex_list)

    d=opengjk.gjk(vertex_list1,vertex_list2)

    if abs(d) < tor:
        return True
    else:
        return False
    


# 检测空间线是否与任何障碍物之间存在干涉
def detect_line_collision(obstacle_list,line,inter=True,tor=1e-4):

    for ob in obstacle_list:
        if detect_polygon_line_collision(ob,line,tor):
            return True 
    
    return False 

# 检测polyhedron与box之间是否存在干涉
def detect_polygon_polygon_collision(ob,poly,tor=1e-4):
    
    
    v1=ob.vertex_list
    v2=poly.vertex_list
    
    d=opengjk.gjk(v1,v2)

    if abs(d) < tor:
        return True
    else:
        return False



def detect_polygon_collision(obstacle_list,poly,tor=1e-4):
    
    for ob in obstacle_list:
        if detect_polygon_polygon_collision(ob,poly,tor):
            return True
    
    return False


def Build_ExtensionZone(obstacle_list,ExtendWidth):

    extend_obstacle_list=[]

    for ob in obstacle_list:

        extend_obstacle_list +=[
            box(ob.x1-ExtendWidth,ob.x2+ExtendWidth,ob.y1-ExtendWidth,ob.y2+ExtendWidth,ob.z1-ExtendWidth,ob.z2+ExtendWidth)
        ]
    
    return extend_obstacle_list

def grid(obstacle_list,resolution,map_range):
    
    start=time.time()

    length=int( (map_range['x'][1]-map_range['x'][0])/resolution )
    width= int( (map_range['y'][1]-map_range['y'][0])/resolution )
    height=int( (map_range['z'][1]-map_range['z'][0])/resolution )

    grid_map=[]

    for x in range(length):

        items=[]

        for y in range(width):
            items+=[[x,y,height,copy.deepcopy(obstacle_list),resolution,copy.deepcopy(map_range)]]

        pool=mp.Pool(20) ## $$ change to the maximum cores of this computer
        grid_map_y_z=pool.map(grid_z, items)
        pool.close()
        pool.join()
        
        grid_map+=[grid_map_y_z]
    grid_map=np.array(grid_map)
    print("Map size is: "+str(length)+" x "+str(width)+" x "+str(height)+" and uses time: "+str(time.time()-start))

    return grid_map

def grid_z(item):

    x,y,height,obstacle_list,res,map_range=item
    x_0=map_range['x'][0]
    y_0=map_range['y'][0]
    z_0=map_range['z'][0]

    grid_map_z=np.zeros(height,dtype=int)
    for z in range(height+1):

        a=detect_point_collision(obstacle_list,\
            np.array([x*res+x_0,y*res+y_0,z*res+z_0]))
        b=detect_point_collision(obstacle_list,\
            np.array([(x+1)*res+x_0,y*res+y_0,z*res+z_0]))
        c=detect_point_collision(obstacle_list,\
            np.array([x*res+x_0,(y+1)*res+y_0,z*res+z_0]))
        d=detect_point_collision(obstacle_list,\
            np.array([(x+1)*res+x_0,(y+1)*res+y_0,z*res+z_0]))

        if a or b or c or d:
            grid_map_z[min(z,height-1)]=1
            grid_map_z[max(0,z-1)]=1
            
    return grid_map_z