
# coding: utf-8

# In[1]:


def GUI_vrep(x,y):
    x = x - 555
    y = y - 505
    
    x = x/100
    y = y/100
    
    return(x,y)
    


# In[2]:


def vrep_GUI(x,y):
    x = x*100
    y = y*100
    
    if x < 0:
        x = 1110 - int(round(abs(x) + 555))
    else:
        x = int(round(abs(x) + 555))
    y = int(round(abs(y) + 505))
    
    return(x,y)


# In[3]:


vrep_GUI(-4,-4)


# In[4]:


import cv2
import numpy 
import copy
import collections
import math
import time
import vrep 
import sys
import random

img_size = [1011,1111]
img = numpy.zeros([1011,1111,3])
print(img.shape)

img[:,:,0] = numpy.ones(img_size)*240.0
img[:,:,1] = numpy.ones(img_size)*240.0
img[:,:,2] = numpy.ones(img_size)*240.0

img = cv2.circle(img,(150,180), 80, (0,0,0), -1)
img = cv2.rectangle(img,(150,100),(310,260),(0,0,0),-1)
img = cv2.circle(img,(310,180), 80, (0,0,0), -1)

img = cv2.circle(img,(390,45), 40, (0,0,0), -1)
img = cv2.circle(img,(438,274), 40, (0,0,0), -1)
img = cv2.circle(img,(438,736), 40, (0,0,0), -1)
img = cv2.circle(img,(390,965), 40, (0,0,0), -1)
#img = cv2.circle(img,(555,505), 10, (0,0,0), -1)

img = cv2.rectangle(img,(438,513),(529,696),(0,0,0),-1)
img = cv2.rectangle(img,(529,660),(712,736),(0,0,0),-1)

img = cv2.rectangle(img,(474,823),(748,975),(0,0,0),-1)
img = cv2.rectangle(img,(685,975),(1110,1010),(0,0,0),-1)
img = cv2.rectangle(img,(779,917),(896,975),(0,0,0),-1)
img = cv2.rectangle(img,(927,899),(1110,975),(0,0,0),-1)

img = cv2.rectangle(img,(832,0),(918,183),(0,0,0),-1)

img = cv2.rectangle(img,(747,313),(1110,389),(0,0,0),-1)
img = cv2.rectangle(img,(983,0),(1026,91),(0,0,0),-1)

img = cv2.rectangle(img,(785,619),(937,736),(0,0,0),-1)
img = cv2.rectangle(img,(1052,445),(1110,562),(0,0,0),-1)
img = cv2.rectangle(img,(1019,562),(1110,648),(0,0,0),-1)
img = cv2.rectangle(img,(1052,715),(1110,832),(0,0,0),-1)


canny_img = cv2.Canny(numpy.uint8(img),100,200)
indices = numpy.where(canny_img != [0])
coordinates = zip(indices[0], indices[1])

for i,j in coordinates:
    img = cv2.circle(img,(j,i), 30, (150,150,150), -1)
    
cv2.namedWindow('RRL_Map.jpg', cv2.WINDOW_NORMAL)        # Create window with freedom of dimensions
cv2.resizeWindow('RRL_Map.jpg', 700, 800)              

#img = cv2.addWeighted(img_clone, 0.3, img, 0.7, 0)

#img = cv2.flip( img, 0 )
cv2.imwrite('RRL_Map.jpg', img)


# In[5]:


def Take_Starting_Node():
    
    img = cv2.imread('RRL_Map.jpg')
    gx = 150
    gy = 150
    while img[gy][gx][0] != 240:
        X = ((input("Give a Starting Node x in range(0,1110) and y in range(0,1010)   ")).split(','))
        #print(X)
        gx = int(X[0])
        gy = int(X[1])
    
        #gx,gy = Goal_Node
        if img[gy][gx][0] == 240:
            print('Given Starting Node is acceptable')
            img = cv2.circle(img,(gx,gy), 2, (0,0,255), -1)

            cv2.imwrite('RRL_Map.jpg', img)
            break
        else:
            print('Given Node is in Obstacle Space, please give a new node')
          
    return(int(X[0]),int(X[1]))


# In[6]:


def Take_Goal_Node():
    img = cv2.imread('RRL_Map.jpg')
    
    gx = 150
    gy = 150
    while img[gy][gx][0] != 240:
        X = ((input("Give a Goal Node x in range(0,1110) and y in range(0,1010)   ")).split(','))
        #print(X)
        gx = int(X[0])
        gy = int(X[1])
    
        if img[gy][gx][0] in range(237,243):
            print('Given Goal Node is acceptable')
            img = cv2.circle(img,(gx,gy), 3, (0,255,0), -1)

            cv2.imwrite('RRL_Map.jpg', img)
            break
        else:
            print('Given Node is in Obstacle Space, please give a new node')
     
    return(int(X[0]),int(X[1]))


# In[7]:


def Mark_Read(x,y):
    
    img = cv2.imread('RRL_Map.jpg')
    img = cv2.circle(img,(x,y), 3, (0,0,0), -1)
    cv2.imwrite('RRL_Map.jpg', img)
    cv2.imshow('RRL_Map.jpg', img)
    cv2.waitKey(1)


# In[8]:


def Mark_Line(node1,node2):
    
    x1,y1 = node1
    x2,y2 = node2.child
    img = cv2.imread('RRL_Map.jpg')
    img = cv2.line(img, (x1,y1),(x2,y2), (0,0,0), 1)
    cv2.imwrite('RRL_Map.jpg', img)
    cv2.imshow('RRL_Map.jpg', img)
    cv2.waitKey(1)


# In[9]:


class Node:

    def __init__(self, child, parent=None,cost = 0,costgo = 0, angle = 0, action = None):
        self.child = child
        self.parent = parent
        self.cost = cost
        self.costgo = costgo
        self.angle = angle
        self.action = action


# In[10]:


def Node_Actions(node,v1,v2,th):
    
    img = cv2.imread('RRL_Map.jpg')
    
    dt = 1
    r = 3.8
    l = 25

    (x,y) = node
    directions = {'RR':((int(round(x + (r/2)*(v1)*math.cos(((r/l)*(v1)*dt)))),int(round(y - (r/2)*(v1)*math.sin(((r/l)*(v1)*dt))))),18.68, -(r/l)*(v1)*dt),
                  'lRR':((int(round(x + (r/2)*(v2)*math.cos(((r/l)*(v2)*dt)))),int(round(y - (r/2)*(v2)*math.sin(((r/l)*(v2)*dt))))),9.21, -(r/l)*(v2)*dt),
                  'RL':((int(round(x + (r/2)*(v1 + v2)*math.cos(((r/l)*(v1 - v2)*dt)))),int(round(y - (r/2)*(v1 + v2)*math.sin(((r/l)*(v1 - v2)*dt))))),28.42, -(r/l)*(v1 - v2)*dt),
                  'S' :((int(round(x + (r/2)*2*(v1)*dt)),y),38, 0),
                  'lS' :((int(round(x + (r/2)*2*(v2)*dt)),y),19, 0),
                  'LR':((int(round(x + (r/2)*(v1 + v2)*math.cos(((r/l)*(v2 - v1)*dt)))),int(round(y + (r/2)*(v1 + v2)*math.sin(((r/l)*(v1 - v2)*dt))))),28.42, (r/l)*(v1 - v2)*dt),
                  'lLL':((int(round(x + (r/2)*(v2)*math.cos(((r/l)*(v2)*dt)))),int(round(y + (r/2)*(v2)*math.sin(((r/l)*(v2)*dt))))),9.21, (r/l)*(v2)*dt),
                  'LL':((int(round(x + (r/2)*(v1)*math.cos(((r/l)*(v1)*dt)))),int(round(y + (r/2)*(v1)*math.sin(((r/l)*(v1)*dt))))),18.68, (r/l)*(v1)*dt)}
   
    valid_direc = {}
    for i in directions.items():
        d,((nx,ny),action_cost,a) = i
        nx, ny = int(round((nx-x)*math.cos((th)) - (ny-y)*math.sin((th)))) + x, int(round((ny-y)*math.cos((th)) + (nx-x)*math.sin((th)))) + y
        #print(nx,ny)
        if nx in range(0,1111) and ny in range(0,1011):
            #print(img[ny][nx])
            if img[ny][nx][0] in range(230,250):
#                 img = cv2.circle(img,(nx,ny), 3, (0,0,0), -1)
#                 cv2.imwrite('RRL_Map.jpg', img)
                valid_direc.update({(d,((nx,ny),action_cost,a))})
    
    return(valid_direc)
    


# In[11]:


def closest_node(node, nodes):
    closest_index = distance.cdist([node], nodes).argmin()
    return nodes[closest_index]


# In[12]:


def Steer(random_point, near_node):

        
        theta = math.atan2(random_point[1] - near_node[1], random_point[0] - near_node[0])
        newNode = Node((random_point[0], random_point[1]))
        #print('newNodes.child before steer',newNode.child)
        currentDistance = math.sqrt(
            (random_point[1] - near_node[1]) ** 2 + (random_point[0] - near_node[0]) ** 2)
        # Find a point within expandDis of nind, and closest to rnd
        if currentDistance <= 10:
            pass
        else:
            newNode.child = (int(near_node[0] + 10*math.cos(theta)), int(near_node[1] + 10*math.sin(theta)))
            
        newNode.cost = float("inf")
        #print('newNodes.child after steer',newNode.child)
        
        return newNode


# In[13]:


def get_random_point():
    x = random.randint(0, 1110)
    y = random.randint(0, 1010)
    return (x,y)


# In[14]:


def GetNearestNode(seen_nodes , randompoint):
    
    dmin = 9999999999
    for node in seen_nodes.values():
        if (node.child[0] - randompoint[0]) ** 2 + (node.child[1] - randompoint[1])** 2 < dmin:
            newnode = node.child
            dmin = (node.child[0] - randompoint[0]) ** 2 + (node.child[1] - randompoint[1])** 2

    return newnode


# In[15]:


def choose_parent(newNode, near_nodes):
        if not near_nodes:
            return newNode
        #print(newNode)
        mincost = 999999999
        mincost_node = []
        for i in near_nodes:
            #print("i", i)
            dx = newNode.child[0] - i.child[0]
            dy = newNode.child[1] - i.child[1]
            d = math.sqrt(dx ** 2 + dy ** 2)
            #print('distance', d)
            theta = math.atan2(dy, dx)
            #if obstacle_line_check(i.child, theta, d):
                #print('First If')
            if i.cost + d < mincost:
                #print('Second If')
                mincost = i.cost + d
                mincost_node = i

        newNode.cost = mincost
        newNode.parent = mincost_node
        #print("parent1:", newNode.parent)

        return newNode


# In[16]:


def Rewire(newNode, nearinds):
       #print('rewire',nearinds)
   
       for i in nearinds:


           dx = newNode.child[0] - i.child[0]
           dy = newNode.child[1] - i.child[1]
           d = math.sqrt(dx ** 2 + dy ** 2)

           scost = newNode.cost + d

           if i.cost > scost:
               theta = math.atan2(dy, dx)
               #if obstacle_line_check(i.child, theta, d):
               i.parent = newNode
               i.cost = scost
       #print(nearinds)  


# In[17]:


def find_near_nodes(newNode,seen_nodes):
    
        nnode = len(seen_nodes)
        r = 50.0 * math.sqrt((math.log(nnode+1) / nnode+1))
        #print('r',r)

        near_node_list = []
        for node in seen_nodes.values():
            #print(node.child)
            d = (node.child[0] - newNode.child[0]) ** 2 + (node.child[1] - newNode.child[1]) ** 2 
            if d < r**2:
                near_node_list.append(node)
                
        return near_node_list


# In[18]:


def obstacle_check(new_node):
    #print('obs check',new_node)
    x,y = new_node
    img = cv2.imread('RRL_Map.jpg')
    
    #print('img',img[int(y)][int(x)])
    if img[int(y)][int(x)][0] in range(230,250):
        return False
    else:
        return True


# In[19]:


def obstacle_line_check(node, theta, d):
    
    tmpNode = copy.deepcopy(node)
    #print(tmpNode)
    tmpNode = list(tmpNode)

    for i in range(int(round(d) / 0.5)):
        tmpNode[0] += 0.5 * math.cos(theta)
        tmpNode[1] += 0.5 * math.sin(theta)
        if not obstacle_check(tmpNode):
            return False

    return True


# In[20]:


def costgo(Start,Goal):
    
    x,y = Start
    m,n = Goal
    return abs(m-x)+abs(n-y)


# In[21]:


def Solve_RRTStar(Start,Goal):
    
    xg,yg = Goal
    
    #start_cost_togo = costgo(Start,Goal)
    start_node = Node(Start,None,0,0)
    Mark_Read(start_node.child[0],start_node.child[1])
    
    flag = True
    queue = collections.deque([start_node])
    seen  = {}
    seen[str((queue[0].child))] = start_node
    
#     V = ((input("Give two wheel speeds   ")).split(','))
#     v1 = int(V[0])
#     v2 = int(V[1])
#     angle = 0
    
    while queue and flag:
        #print(seen)
        Prandom = get_random_point()
        near_node = GetNearestNode(seen, Prandom)
        #print('near node',near_node)
        new_node = Steer(Prandom, near_node)
        #print('new_node',new_node.child)
        
        if not obstacle_check(new_node.child):
            
            if (math.sqrt((new_node.child[0] - xg)**2 + (new_node.child[1] - yg)**2) <= 20):
                print('Goal Node Reched')
                Goal_Node = Node(new_node.child)
                near_nodes = find_near_nodes(new_node,seen)
                Goal_Node = choose_parent(new_node, near_nodes)
                Rewire(Goal_Node, near_nodes)
                Mark_Read(Goal_Node.child[0],Goal_Node.child[1])
                flag = False
                break
                
            near_nodes = find_near_nodes(new_node,seen)
#             for i in near_nodes:
#                 print('near_nodes',i.child)
            
            #print("new node:", new_node.child)
            new_node = choose_parent(new_node, near_nodes)
#             print('new node parent2',new_node.parent)
#             print('new node child2',new_node.child)
            Rewire(new_node, near_nodes)
            seen[(str(new_node.child))] = new_node
            Mark_Read(new_node.child[0],new_node.child[1])
#             for i in near_nodes:
#                 print('near_nodes2',i.child)
            
            #print("new node2:", new_node.child)
            Mark_Line(new_node.child, new_node.parent)
            
        
    P = []
    prev_node = Goal_Node.child
    P.append(Goal_Node.child)
    while Goal_Node.child != Start and Goal_Node.parent:
        print('Goal_node parent',Goal_Node.parent)
        Goal_Node = Goal_Node.parent
        p,q = Goal_Node.child
        print('Goal_node child',Goal_Node.child)
        P.append(Goal_Node.child)
        #print(P)
        
        img = cv2.imread('RRL_Map.jpg')
        img = cv2.circle(img,(p,q), 3, (0,255,0), -1)
        img = cv2.line(img,prev_node,(p,q),(0,255,0),1)
        prev_node = (p,q)
        cv2.imwrite('RRL_Map.jpg', img)
        cv2.imshow('RRL_Map.jpg', img)
        cv2.waitKey(1)

    cv2.destroyAllWindows


# In[22]:


Start = Take_Starting_Node()
Goal = Take_Goal_Node()


# In[23]:


Solve_RRTStar(Start,Goal)

