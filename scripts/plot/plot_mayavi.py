import numpy as np
import mayavi.mlab as mlab
import plot.color_list as color_list

color=color_list.color_RGB
jet_list = color_list.jet_list

def plot_sphere(p,r,c):
    
    x=p[0]
    y=p[1]
    z=p[2]
    mlab.points3d(x, y, z, r*2, scale_factor=1, resolution=10, mode="sphere",color=c)

def plot_3D(x,y,z,c):

    if len(x)!=len(y):
        raise Exception("the length of x and y is not equal")

    mlab.plot3d(x,y,z,color=c)
    

def plot_polygon(polygon,color):

    v_list=polygon.vertex_list
    x=[v[0] for v in v_list]
    y=[v[1] for v in v_list]
    z=[v[2] for v in v_list]
    tri=[]
    l=len(v_list)
    for i in range(l-2):
        for j in range(i+1,l-1):
            for k in range(j+1,l):
                tri+=[[i,j,k]]
    
    mlab.triangular_mesh(x,y,z,tri,color=color)

    

def plot_cube(box,color):

    if box.type == 'box_line':

        x=[box.x1,box.x2]
        y=[box.y1,box.y2]
        z=[box.z1,box.z2]

        mlab.plot3d(x,y,z,tube_radius=0.1,color=color)

    else:

        x=(box.x1+box.x2)/2
        y=(box.y1+box.y2)/2
        z=(box.z1+box.z2)/2
        dx=box.x2-box.x1
        dy=box.y2-box.y1
        dz=box.z2-box.z1

        xx = np.linspace(x-dx/2, x+dx/2, 2)
        yy = np.linspace(y-dy/2, y+dy/2, 2)
        zz = np.linspace(z-dz/2, z+dz/2, 2)

        xm, ym = np.meshgrid(xx, yy)
        mlab.mesh(xm, ym, (z-dz/2)*np.ones((2,2)),color=color)
        mlab.mesh(xm, ym, (z+dz/2)*np.ones((2,2)),color=color)

        ym, zm = np.meshgrid(yy, zz)
        mlab.mesh((x-dx/2)*np.ones((2,2)), ym, zm ,color=color)
        mlab.mesh((x+dx/2)*np.ones((2,2)), ym, zm ,color=color)

        xm, zm = np.meshgrid(xx, zz)
        mlab.mesh(xm, (y-dy/2)*np.ones((2,2)), zm ,color=color)
        mlab.mesh(xm, (y+dy/2)*np.ones((2,2)), zm ,color=color)
    
def plot_ini_x(p,c):

    x=p[0]
    y=p[1]
    z=p[2]
    dx=15
    dy=15
    dz=15

    xx = np.linspace(x-dx/2, x+dx/2, 2)
    yy = np.linspace(y-dy/2, y+dy/2, 2)
    zz = np.linspace(z-dz/2, z+dz/2, 2)

    xm, ym = np.meshgrid(xx, yy)
    mlab.mesh(xm, ym, (z-dz/2)*np.ones((2,2)),color=c)
    mlab.mesh(xm, ym, (z+dz/2)*np.ones((2,2)),color=c)

    ym, zm = np.meshgrid(yy, zz)
    mlab.mesh((x-dx/2)*np.ones((2,2)), ym, zm ,color=c)
    mlab.mesh((x+dx/2)*np.ones((2,2)), ym, zm ,color=c)

    xm, zm = np.meshgrid(xx, zz)
    mlab.mesh(xm, (y-dy/2)*np.ones((2,2)), zm ,color=c)
    mlab.mesh(xm, (y+dy/2)*np.ones((2,2)), zm ,color=c)

def plot_target(p,c):

    x=p[0]
    y=p[1]
    z=p[2]
    dx=15
    dy=15
    dz=22.5

    xx = np.linspace(x-dx/2, x+dx/2, 3)
    yy = np.linspace(y+dy/2, y-dy/2, 3)


    xm, ym = np.meshgrid(xx, yy)
    xmym=np.array([2*abs(xm-x)/dx,2*abs(ym-y)/dy])
    zm=z+dz/2*(1-xmym.max(axis=0))
    mlab.mesh(xm, ym, zm,color=c)
    zm=z-dz/2*(1-xmym.max(axis=0))
    mlab.mesh(xm, ym, zm,color=c)

    xx = np.linspace(x-dx/2, x+dx/2, 3)
    yy = np.linspace(y-dy/2, y+dy/2, 3)


    xm, ym = np.meshgrid(xx, yy)
    xmym=np.array([2*abs(xm-x)/dx,2*abs(ym-y)/dy])
    zm=z+dz/2*(1-xmym.max(axis=0))
    mlab.mesh(xm, ym, zm,color=c)
    zm=z-dz/2*(1-xmym.max(axis=0))
    mlab.mesh(xm, ym, zm,color=c)
    

def plot_obstacle(obstacle_list,extend=False):

    
    for i in range(31):

        mlab.plot3d([-500,1000],[-525+50*i,-525+50*i],[0,0],tube_radius=0.01,color=(0.7,0.7,0.7))
        mlab.plot3d([-500+50*i,-525+50*i],[-525,1000],[0,0],tube_radius=0.01,color=(0.7,0.7,0.7))
    


    bg_color=(1,1,1)
    color=(150/255,150/255,150/255)

    i=0
    for ob in obstacle_list:
        if i==0:
            # plot_polygon(ob,bg_color)
            pass
        else:
            plot_polygon(ob,(color[0]+0.02*i,color[1]+0.02*i,color[2]+0.02*i))
        i+=1
    
        


def plot_connect(connect_list,agent_list):

    for connect in connect_list:

        p1=agent_list[connect[0]].p
        p2=agent_list[connect[1]].p

        x=[p1[0],p2[0]]
        y=[p1[1],p2[1]]
        z=[p1[2],p2[2]]

        mlab.plot3d(x,y,z,tube_radius=1.0,color=(122/255,197/255,255/255))



def plot_pre_traj(
            agent_list,
            obstacle_list,
            plot_range,
            episodes,
            format,
            show,
            connect_list=[],
            target_list=[]
            ):


    if not show:
        return None
    
    # if episodes < 34:
    #     return None
        

    mlab.figure(size=(2000,2000),bgcolor=(255/255,255/255,255/255)) 

    i = 0
    for agent in agent_list:
        
        if agent.type=="Anchor":
            plot_ini_x(agent.p,c=(1.0,0.0,0.0))
            i+=1
            continue
        else:    
            plot_sphere(agent_list[i].pre_traj[0],r=6.0,c=color[i])

        mlab.plot3d(agent_list[i].pre_traj[:,0],agent_list[i].pre_traj[:,1],agent_list[i].pre_traj[:,2],tube_radius=1.4,color=color[i])
        
        
        x=[agent_list[i].pre_traj[-1][0],agent_list[i].tractive_point[0]]
        y=[agent_list[i].pre_traj[-1][1],agent_list[i].tractive_point[1]]
        z=[agent_list[i].pre_traj[-1][2],agent_list[i].tractive_point[2]]
        mlab.plot3d(x,y,z,tube_radius=0.5,colormap='Spectral')
    
        i+=1

    for tar  in target_list:

        plot_target(tar,c=(1.0,0.5,0.5))


    plot_connect(connect_list,agent_list)

    plot_obstacle(obstacle_list)

    # print(mlab.view( ))
    mlab.view(-100.0, 20.0, 990.00, np.array([250, 220.0, 0.0]))

    mlab.savefig('savefig/episode-'+str(episodes)+format)

    # mlab.show()
    mlab.close()

    return None

# plot
def plot_position(
        agent_list,
        obstacle_list,
        plot_range=[],
        connect_list=[],
        format='jpg',
        target_list=[]
        ):
    

    mlab.figure(size=(2000,2000),bgcolor=(255/255,255/255,255/255)) 
    
    plot_obstacle(obstacle_list)
    
    i=0
    for agent in agent_list:
        if agent.type=="Anchor":
            plot_ini_x(agent.p,c=(1.0,0.0,0.0))
            continue
        else:    
            plot_sphere(agent.p,r=7.0,c=color[i])

        
        mlab.plot3d(agent.position[:,0],agent.position[:,1],agent.position[:,2],color=color[i],tube_radius=1.0,opacity=0.5)
        i+=1

        # mlab.plot3d(agent_list[i].position[:,0],agent_list[i].position[:,1],agent_list[i].position[:,2])
        # plot_ini_x(agent_list[i].position[0],c=color[i])

    for tar  in target_list:

        plot_target(tar,c=(1.0,0.5,0.5))

    plot_connect(connect_list,agent_list)

    mlab.view(-100.0, 20.0, 990.00, np.array([250, 220.0, 0.0]))
    mlab.savefig('savefig/trajecotry'+format)
    mlab.show()
    # mlab.close()


def plot_path_tree(
        path_tree,
        obstacle_list,
        path_list=[],
        target = [],
        format='jpg',
        num=0):


    mlab.figure(size=(2000,2000),bgcolor=(255/255,255/255,255/255))     
    # mlab.figure(size=(2000,2000),bgcolor=(240/255,248/255,255/255))     
    
    plot_obstacle(obstacle_list,extend=False)

    plot_ini_x([10.0,10.0,5.0],c=(1.0,0.0,0.0))

    i=0
    for path in path_tree:
        
        mlab.plot3d(path[:,0],path[:,1],path[:,2],tube_radius=1.8,color=color[i])
        for p in path:
            plot_sphere(p,4.0,c=color[i])
        i=i+1

    i = 0
    
    for tar  in target:

        plot_target(tar,c=(1.0,0.5,0.5))
        i=i+1
    
    mlab.view(-100.0, 20.0, 990.00, np.array([250, 200.0, 0.0]))

    mlab.savefig('savefig/path_tree_'+str(num)+format)

    mlab.show()
    # mlab.close()