
import numpy as np
import cv2
from math import *
import matplotlib.path as mplPath
import sys
import imutils
#Clearance
c=5
# Here we have defined clearance between obstacles and 
#Point robot
##################################
#section 1 :
############ Define obstacles in map
#Define governing eqations for points inside the obstacle
#Color the obstacles as blue


def obstacle_circle(map):
  crow= 300+c
  ccol= 185+c
  rad= 40
  for row in range(400):
    for col in range(250):
      if (row- crow)**2+(col-ccol)**2- rad**2< 0:
        map[row][col]=(255,0,0)
  return map    
      
    
def polygon(map):
   x1,y1=36+c,185+c
   x2,y2=115+c,210+c
   x3,y3=80+c,180+c
   x4,y4=105+c,100+c
   for row in range(400):
     for col in range(250):
      r1 = (col - y1) * (x2 -x1) - (row - x1) * (y2 - y1)
      r2 = (col - y2) * (x3 -x2) - (row - x2) * (y3 - y2) 
      r3 = (col - y3) * (x4 -x3) - (row - x3) * (y4 - y3) 
      r4 = (col - y4) * (x1 -x4) - (row - x4) * (y1 - y4) 
      r5 = (col - y1) * (x3 -x1) - (row - x1) * (y3 - y1) 
      if (r1<=0 and r2<=0 and r5>=0) or (r3<=0 and r4<=0 and r5<=0):
        map[row][col]=(255,0,0)
   return map
def hexagon(map):
   s=40.4125
   x1,y1=200+c,100+s+c
   x2,y2=235+c,100+s/2+c
   x3,y3=235+c,100-s/2+c
   x4,y4=200+c,100-s+c
   x5,y5=165+c,100-s/2+c
   x6,y6=165+c,100+s/2+c
   for row in range(400):
     for col in range(250):
      r1 = (col - y1) * (x2 -x1) - (row - x1) * (y2 - y1)
      r2 = (col - y2) * (x3 -x2) - (row - x2) * (y3 - y2) 
      r3 = (col - y3) * (x4 -x3) - (row - x3) * (y4 - y3) 
      r4 = (col - y4) * (x5 -x4) - (row - x4) * (y5 - y4) 
      r5 = (col - y5) * (x6 -x5) - (row - x5) * (y6 - y5) 
      r6 = (col - y6) * (x1 -x6) - (row - x6) * (y1 - y6) 
      if r1<=0 and r2<=0 and r3<=0 and r4<=0 and r5<=0 and r6<=0 :
        map[row][col]=(255,0,0)
   return map

#checks if next node is safe
def check_safe(map,x,y):
    if x<=395 and y<=245 and x>=5 and y>=5 and  (np.int0(map[x][y])==(255,255,255)).all(): 
        return True
    else:
      return False    
#finds neighbors      
def find_neighbors(world,current_node):
    x=current_node[0]
    y=current_node[1]
    neighbor=[]
    c1=1
    c2=sqrt(2)
    #west
    if check_safe(world,x-1,y):
      neighbor.append([(x-1,y),c1])
    #east
    if check_safe(world,x+1,y):
      neighbor.append([(x+1,y),c1])
    #south
    if check_safe(world,x,y+1):
      neighbor.append([(x,y+1),c1])
    #north
    if check_safe(world,x,y-1):
      neighbor.append([(x,y-1),c1])  
    #north_west
    if check_safe(world,x-1,y-1):
      neighbor.append([(x-1,y-1),c2]) 
    #  north_east
    if check_safe(world,x+1,y-1):
      neighbor.append([(x+1,y-1),c2])  
    #south_west 
    if check_safe(world,x-1,y+1):
      neighbor.append([(x-1,y+1),c2]) 
    #south_east
    if check_safe(world,x+1,y+1):
      neighbor.append([(x+1,y+1),c2])  
    return neighbor 
def resize_function(mat, angle):
   

    height, width = mat.shape[:2] 
    image_center = (width/2, height/2) 

    rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1.)

    
    abs_cos = abs(rotation_mat[0,0]) 
    abs_sin = abs(rotation_mat[0,1])

    # find the new width and height bounds
    bound_w = int(height * abs_sin + width * abs_cos)
    bound_h = int(height * abs_cos + width * abs_sin)

    # subtract old image center (bringing image back to origo) and adding the new image center coordinates
    rotation_mat[0, 2] += bound_w/2 - image_center[0]
    rotation_mat[1, 2] += bound_h/2 - image_center[1]

    # rotate image with the new bounds and translated rotation matrix
    rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h))
    return rotated_mat

#Main function which gives visited nodes and path
def dijktras(world,start,goal):
    open_list=[] #list of open nodes
    
    #For neat visualization use closed_list however using a simple list will make alogorithm
    #time complex
    closed_list=[] 
    #using set( ) for checking in closed list will significantly reduce the cost
    #closed_list=set()
    parents=dict() # dictionary for storing parent nodes
    goal_costs=dict() # dictionary
    goal_costs[start]=0 #coordinates are the key. Coordinates are tuple
    open_list.append([start,0])
    shortest_path = []
    path_found = False #Flag for path
    while open_list:
        open_list.sort(key = lambda x: x[1]) #sort the list 
        current_node = open_list.pop(0)[0] # pop the first node
        closed_list.append(current_node)
        #closed_list.add(current_node)
        if current_node==goal:
            path_found=True
            break
        neighbors=find_neighbors(world, current_node)
       
        if neighbors is None:
            continue

        for neighbor,stepcost in neighbors:
            
               
            if neighbor in closed_list :
                continue
            g_cost=goal_costs.get(current_node)+stepcost
            in_open_list=False
            
            for idx,element in enumerate(open_list):
                
              
                if element[0]==neighbor:
                    in_open_list=True
                    break
          
            if in_open_list:
                if g_cost<goal_costs.get(neighbor):
                   
                  
                   goal_costs[neighbor]=g_cost
                   parents[neighbor] = current_node
                   open_list[idx]=[neighbor,g_cost]
            else:
                goal_costs[(neighbor)] = g_cost
                parents[(neighbor)] =current_node
                open_list.append([neighbor, g_cost])
    if not path_found:
        return shortest_path      
        

    if path_found:
      node = goal
      shortest_path.append(goal)
      while node != start:
          shortest_path.append(node)
         
          node = parents[node]
    shortest_path = shortest_path[::-1]    
    return shortest_path,closed_list      
map= 255*np.ones((400,250,3))

map=obstacle_circle(map)
map=polygon(map)
map=hexagon(map)
map_=map.copy()

x_s = int(input("Enter x start coordinate greater than 5 and less 395 : "))
y_s=int(input("Enter y start coordinate  greater than 5 and less 195:"))

start=(x_s,y_s)


if  not check_safe(map, start[0],start[1]):
    sys.exit("start node inside object or out of limit. Please enter again")

x_g = int(input("Enter x goal coordinate greater than 5 and less 395 : "))
y_g= int(input("Enter y goal coordinate greater than 5 and less 395:"))

goal=(x_g,y_g)


if  not check_safe(map, goal[0],goal[1]):
    sys.exit("Goal node inside object or out of limit")    
    
path,closed_list=(dijktras(map_, start, goal))
print("Path found:",path)
print("-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-")
print("Exploring using set for storing closed nodes. ")
img=map.copy()

for v in closed_list:
                
                x=v[0]
                y=v[1]
                img[x][y]=(0,0,0)
                rotate=resize_function(img,90)
                cv2.imshow('map',rotate)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break


print("--------------------------")  
print("Plotting the path")                   
for p in path:
      x=p[0]
      y=p[1]           
      img[x][y]=(0,0,255)
      rotate=resize_function(img,90)
      cv2.imshow('map', rotate)
      cv2.waitKey(150)

cv2.imwrite("Final_solution.png", img)      
cv2.destroyAllWindows()