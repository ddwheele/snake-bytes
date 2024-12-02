#!/usr/bin/python

import sys
import numpy as np
import math
import yaml
from queue import PriorityQueue

MAX_FLOAT = sys.float_info.max
MAP = {}
MAP2 = {}
X_SPACING = 1
Y_SPACING = 1

class Node:
    # dist is distance to start node
    # point is a 2-elem tuple in Grid coordinates
    # parent is the Node before this in the path
    def __init__(self, point, dist=MAX_FLOAT, parent=None):
        self.point = point
        self.parent = parent
        self.distance = dist

    def print_everything(self):
        print("---")
        xy = grid_to_real(self.point)
        print(f"Grid: {self.point}, Real {xy}")
        print("Distance: " + str(self.distance))
        if self.parent != None:
            print("Parent " + self.parent.to_string())
        else:
            print("No Parent")
        print("---")

    # returns the grid distance to the other Node
    def distance_to(self, otherNode):
        oi = otherNode.point[0]
        oj = otherNode.point[1]
        mi = self.point[0]
        mj = self.point[1]
       # print(str(mi) + "," + str(mj) + " to " + str(oi) + "," + str(oj) + " is "  + str(np.sqrt((oi-mi)**2 + (oj-mj)**2)))
        return np.sqrt((oi-mi)**2 + (oj-mj)**2)

    def as_real_numpy(self):
        x = (self.point[1] + 0.5)*X_SPACING
        y = (self.point[0] + 0.5)*Y_SPACING
        return np.array([x, y])

    def to_string(self):
        return f"{self.point}"

    def print_me_grid(self):
        print("---")
        print(f"Node grid coords: {self.point}")
        print("my distance is " + str(self.distance))
        print("---")
       
    def print_me_real(self):
        xy = grid_to_real(self.point)
        print(f"Node real coords: {xy}")
        print("my distance is " + str(self.distance))

def mark_on_map(ij, symbol):
    global MAP2
    i = ij[0]
    j = ij[1]
    print("marking map at " + str(i) + ", " + str(j) + " with " + symbol)
    MAP2[i][j] = symbol

def print_map1():  
    size = np.shape(MAP)
    print("size of MAP is " + str(size))
    max_i = size[0]
    max_j = size[1]
    for j in range(0, max_j):
        for i in range(0, max_i):
            print(MAP[i][j], end=' ')
        print("")

def print_map2():  
    size = np.shape(MAP2)
    print("size of MAP2 is " + str(size))
    max_i = size[0]
    max_j = size[1]
    for j in range(0, max_j):
        for i in range(0, max_i):
            print(MAP2[i][j], end=' ')
        print("")

def grid_square_in_bounds(ij):
    i = ij[0]
    j = ij[1]
    size = np.shape(MAP)
    print("size of MAP is " + str(size))
    max_i = size[0]
    max_j = size[1]
    print("ij is " + str(ij))
    if i < 0 or j < 0  or  i >= max_i or j >= max_j:
        return False
    return True

# return true if map square is available, false otherwise
def grid_square_fair(ij):
    if not grid_square_in_bounds(ij):
        return False
    
    i = ij[0]
    j = ij[1]
    val = MAP[i][j]
    if val > 0:
        return False
    else:
        return True

def real_to_grid(xy):
    j = xy[0]/X_SPACING - 0.5
    i = xy[1]/Y_SPACING - 0.5
    return (i, j)

def grid_to_real(ij):
    x = (ij[1] + 0.5)*X_SPACING
    y = (ij[0] + 0.5)*Y_SPACING
    return (x, y)

def grid_to_real_numpy(ij):
    x = (ij[1] + 0.5)*X_SPACING
    y = (ij[0] + 0.5)*Y_SPACING
    return np.array([[x], [y], [0]])

def find_nearest_grid(xy):
    ij = real_to_grid(xy)

    near_i = round(ij[0][0])
    near_j = round(ij[1][0])
    return (near_i, near_j)

def construct_path(real_start_node, real_goal_node):
    # Intermediate list to store coordinates
    intermediate_structure = []
    curr_node = real_goal_node
    count = 0
  #  print("+++++ here is the list:")
    while not curr_node == real_start_node:
        #curr_node.print_everything()
        intermediate_structure.insert(0, curr_node.as_real_numpy())
        curr_node = curr_node.parent
        count = count + 1

    # now curr_node is real_start_node
   # curr_node.print_everything()
    intermediate_structure.insert(0, curr_node.as_real_numpy())
    count = count + 1
    
   # print("+++++++++++++ end the list:")
 #   print("\nreal_goal_node distance is " + str(real_goal_node.distance))
 #   print("count = "+ str(count))

    num = len(intermediate_structure)
  #  print("num is " + str(num)) 

    num  = count
   
    path = np.zeros((num, 2))
   # print("Just initialized as zeros()")
   # print(path)
    curr_node = real_goal_node
   # real_goal_node.print_everything()
    path[num-1,:] = real_goal_node.as_real_numpy()
   # print(path)
    i=num-2
    while i > 0:
      #  print("i = " + str(i))
        curr_node = curr_node.parent
      #  curr_node.print_everything()
        path[i,:] = curr_node.as_real_numpy()
        i=i-1
        if i>0:
            mark_on_map(curr_node.point, "=,=" )#+ str(curr_node.distance))
        #print_map2()
        #print(path)
        #input("Press Enter to continue...")

    path[0,:] = real_start_node.as_real_numpy()
    print(path)
    print_map2()
    return path

# add to queue if it's unobstructed and has a shorter distance
def evaluate_this_neighbor(node_q, current_node, neighbor_node, curr_dist):
    print("  evaluating neighbor:" + neighbor_node.to_string())
    
    if not grid_square_fair(neighbor_node.point):
      #  print("   -rejecting, it was occupied")
        return node_q
    potential_new_distance = curr_dist + current_node.distance_to(neighbor_node)
    if potential_new_distance < neighbor_node.distance:
     #   print("   -now it's at distance " + str(potential_new_distance))
        neighbor_node.parent = current_node
     #   print("The parent of " + neighbor_node.to_string() + " is  "+ current_node.to_string())
        neighbor_node.distance = potential_new_distance
        mark_on_map(neighbor_node.point, "e, " )
        # add it to the priority queue with new distance
        node_q.put((potential_new_distance, neighbor_node.point))
   # else:
     #   print("   Potential new dist=" + str(potential_new_distance) + ", old dist=" + str(neighbor_node.distance))
    
    # return the modified queue
    return node_q

def add_first_neighbors(node_dict, node_q, real_start_node):
    si = math.floor(real_start_node.point[0])
    sj = math.floor(real_start_node.point[1])
   # print("in add neighbors, " + str(si)+ ", " + str(sj))
    a = (si, sj)
    b = (si+1, sj)
    c = (si, sj+1)
    d = (si+1, sj+1)

    if grid_square_in_bounds(a):
        aNode = node_dict[a]
        node_q = evaluate_this_neighbor(node_q, real_start_node, aNode, 0)
    
    if grid_square_in_bounds(b):
        bNode = node_dict[b]
        node_q = evaluate_this_neighbor(node_q, real_start_node, bNode, 0)

    if grid_square_in_bounds(c):
        cNode = node_dict[c]
        node_q = evaluate_this_neighbor(node_q, real_start_node, cNode, 0)
    
    if grid_square_in_bounds(d):
        dNode = node_dict[d]
        node_q = evaluate_this_neighbor(node_q, real_start_node, dNode, 0)

   # print(f"neighbors are {a}, {b}, {c}, {d}")
  #  print(f"distances are {real_start_node.distance_to(aNode)}, {real_start_node.distance_to(bNode)}, {real_start_node.distance_to(cNode)}, {real_start_node.distance_to(dNode)}")
    return node_q

def process_neighbor(node_q, node_dict, current_node, neighbor_coord, current_node_dist):
    print(f"Processing neighbor {neighbor_coord}")
    if(grid_square_in_bounds(neighbor_coord)):
        neighbor_node = node_dict[neighbor_coord]
        print(" that is in bounds: " + neighbor_node.to_string())
        node_q = evaluate_this_neighbor(node_q,current_node,neighbor_node,current_node_dist)
    else:
        print(" -rejecting, it was out of bounds")
    return node_q

def update_goal_node_if_neighbor_is_near_it(real_goal_node, node_dict, neighbor_coord, current_node_dist):
    neighbor_node = node_dict[neighbor_coord]
    neighbor_dist = real_goal_node.distance_to(neighbor_node)
    if (nodes_are_close(neighbor_node, real_goal_node)):
        real_goal_node.parent = neighbor_node
        real_goal_node.distance = current_node_dist + 1 + neighbor_dist
    return real_goal_node

def dijkstras(start,goal):
    """
    Implements Dijkstra's shortest path algorithm
    Input:
    occupancy_map - an N by M numpy array of boolean values (represented
        as integers 0 and 1) that represents the locations of the obstacles
        in the world
    x_spacing - parameter representing spacing between adjacent columns
    y_spacing - parameter representing spacing between adjacent rows
    start - a 3 by 1 numpy array of (x,y,theta) for the starting position 
    goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position 
    Output: 
    path: list of the indices of the nodes on the shortest path found
        starting with "start" and ending with "end" (each node is in
        metric coordinates)
    """
    st = real_to_grid(start)
    gl = real_to_grid(goal)
    start_frac_grid = (st[0][0], st[1][0])
    goal_frac_grid = (gl[0][0], gl[1][0])

    print("============================================")
    print(f"start = {start}")
    print(f"goal = {goal}")
    print(f"start frac grid = {start_frac_grid}")
    print(f"goal frac grid = {goal_frac_grid}")

    real_start_node = Node(start_frac_grid,0)
    real_goal_node = Node(goal_frac_grid)

    print("real_start_node = ")
    real_start_node.print_everything()
    print("real_goal_node = ")
    real_goal_node.print_everything()

    # dictionary with key coordinate tuple and value Node
    node_dict = {}
    # premake all the dictionary entries
    size = np.shape(MAP)
    max_i = size[0]
    max_j = size[1]
    for j in range(0, max_j):
        for i in range(0, max_i):
            node_dict[(i,j)] = Node((i,j))

    # priority queue linking distance from start to grid coordinates
    node_q = PriorityQueue()

    ## Find the nearest neighbors to the start node and add to queue
    node_q = add_first_neighbors(node_dict, node_q, real_start_node)

    ## Find the nearest node to the end node
    near_start_coord = find_nearest_grid(start)
    near_goal_coord = find_nearest_grid(goal)

    print("start is near to " + str(near_start_coord[0]) + ", " + str(near_start_coord[1]))
    print("goal is near to " + str(near_goal_coord[0]) + ", " + str(near_goal_coord[1]))

    best_q_entry = node_q.get()       
    curr_coords = best_q_entry[1]
    print(f"THE START is {curr_coords}")

    mark_on_map(near_start_coord, 'S,0')
    mark_on_map(near_goal_coord, 'G,?')
    print_map2()
    
    goal_coord = near_goal_coord
    while(True):
        # Take the node with the smallest distance
        curr_i = curr_coords[0]
        curr_j = curr_coords[1]
        current_node = node_dict[curr_coords]

        #print("current_node = ")
       # current_node.print_everything()
        current_node_dist = current_node.distance
        #print("current_node_dist=" + str(current_node_dist))
        # find the neighbors of current_node
        north_coord = (curr_i-1, curr_j)
        if grid_square_in_bounds(north_coord):
            node_q = process_neighbor(node_q, node_dict, current_node, north_coord, current_node_dist)
            real_goal_node = update_goal_node_if_neighbor_is_near_it(real_goal_node, node_dict, north_coord, current_node_dist)
            if not real_goal_node.parent == None:
                break
        
        south_coord = (curr_i+1, curr_j)
        if grid_square_in_bounds(south_coord):
            node_q = process_neighbor(node_q, node_dict, current_node, south_coord, current_node_dist)
            real_goal_node = update_goal_node_if_neighbor_is_near_it(real_goal_node, node_dict, south_coord, current_node_dist)
            if not real_goal_node.parent == None:
                break

        east_coord = (curr_i, curr_j-1)
        if grid_square_in_bounds(east_coord):
            node_q = process_neighbor(node_q, node_dict, current_node, east_coord, current_node_dist)
            real_goal_node = update_goal_node_if_neighbor_is_near_it(real_goal_node, node_dict, east_coord, current_node_dist)
            if not real_goal_node.parent == None:
                break

        west_coord = (curr_i, curr_j+1)
        if grid_square_in_bounds(west_coord):
            node_q = process_neighbor(node_q, node_dict, current_node, west_coord, current_node_dist)
            real_goal_node = update_goal_node_if_neighbor_is_near_it(real_goal_node, node_dict, west_coord, current_node_dist)
            if not real_goal_node.parent == None:
                break

        # Visit all of its neighbors and add them with the distance to the q
       # print_map2()
       # input("Press Enter to continue...")
        best_q_entry = node_q.get()
        curr_coords = best_q_entry[1]

    print("I found a path!!!!!!!!!!!!!!!!")

    # finish the last node
    print(f"reached goal {real_goal_node.point}")
    # goal_node = node_dict[goal_coord]

    mark_on_map(goal_coord, "G," + str(real_goal_node.parent))
    path = construct_path(real_start_node, real_goal_node)
    return path

def nodes_are_close(n1, n2) :
    if(abs(n1.point[0] - n2.point[0]) < 1):
        if(abs(n1.point[1] - n2.point[1]) < 1):
            return True
    return False

def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
    test_map0 = np.array([
              [1, 0, 0, 0, 0, 0, 0, 1],
              [0, 0, 0, 0, 0, 0, 0, 0],
              [1, 0, 0, 0, 0, 0, 1, 1]])
    x_spacing0 = 1
    y_spacing0 = 1
    start0 = np.array([[1.1], [1.1], [0]])
    goal0 = np.array([[6], [1.1], [0]])
    path0 = call_me(test_map0,x_spacing0,y_spacing0,start0,goal0)
    s = 0
    for i in range(len(path0)-1):
      s += np.sqrt((path0[i][0]-path0[i+1][0])**2 + (path0[i][1]-path0[i+1][1])**2)


    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.13
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.3], [0]])
    goal1 = np.array([[0.6], [1], [0]])
    path1 = call_me(test_map1,x_spacing1,y_spacing1,start1,goal1)
    true_path1 = np.array([
        [ 0.3  ,  0.3  ],
        [ 0.325,  0.3  ],
        [ 0.325,  0.5  ],
        [ 0.325,  0.7  ],
        [ 0.455,  0.7  ],
        [ 0.455,  0.9  ],
        [ 0.585,  0.9  ],
        [ 0.600,  1.0  ]
        ])
    if np.array_equal(path1,true_path1):
      print("Path 1 passes")

    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)

    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
    print("Path 1 length:")
    print(s)
    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")

    test_map2 = np.array([
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.5], [1.0], [1.5707963267948966]])
    goal2 = np.array([[1.1], [0.9], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = call_me(test_map2,x_spacing2,y_spacing2,start2,goal2)
    true_path2 = np.array([[ 0.5,  1.0],
                           [ 0.5,  1.1],
                           [ 0.5,  1.3],
                           [ 0.5,  1.5],
                           [ 0.7,  1.5],
                           [ 0.9,  1.5],
                           [ 1.1,  1.5],
                           [ 1.1,  1.3],
                           [ 1.1,  1.1],
                           [ 1.1,  0.9]])
    if np.array_equal(path2,true_path2):
      print("Path 2 passes")
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
    print("Path 2 length:")
    print(s)
    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")


def test_for_grader():
    """
    Function that provides the test paths for submission
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 1, 0, 0, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 0, 0, 1, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 1
    y_spacing1 = 1
    start1 = np.array([[1.5], [1.5], [0]])
    goal1 = np.array([[7.5], [1], [0]])
    path1 = call_me(test_map1,x_spacing1,y_spacing1,start1,goal1)
    
    s = 0
    for i in range(len(path1)-1):
        s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
    print("Path 1 length:")
    print(s)
    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")

    test_map2 = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.4], [0.4], [1.5707963267948966]])
    goal2 = np.array([[0.4], [1.8], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = call_me(test_map2,x_spacing2,y_spacing2,start2,goal2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
    print("Path 2 length:")
    print(s)
    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")

def call_me(test_map1,x_spacing1,y_spacing1,start1,goal1):
    global MAP
    global MAP2
    global X_SPACING
    global Y_SPACING

    MAP = test_map1
    X_SPACING = x_spacing1
    Y_SPACING = y_spacing1

    size = np.shape(MAP)
    print("map size = " + str(size))
    max_i = size[0] # 3
    max_j = size[1] # 8
    MAP2 = [[" . " for i in range(max_j)] for j in range(max_i)]

    print_map1()
    print_map2() 
    for j in range(0, max_j):
        for i in range(0, max_i):
            if MAP[i][j] > 0:
                MAP2[i][j] = "111"

    return dijkstras(start1, goal1)


def main():
    global MAP
    global MAP2
    global X_SPACING
    global Y_SPACING

    # Load parameters from yaml
    param_path = 'params.yaml' # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw, Loader=yaml.Loader)
    # Get params we need
    # occupancy_map = np.array(params['occupancy_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])

    print("pos_init is " + str(pos_init[0]) + ", " + str(pos_init[1]))
    print("pos_goal is " + str(pos_goal[0]) + ", " + str(pos_goal[1]))
    #x_spacing = params['x_spacing']
    #y_spacing = params['y_spacing']
    X_SPACING = params['x_spacing']
    Y_SPACING = params['y_spacing']
    MAP = np.array(params['occupancy_map'])
 
    size = np.shape(MAP)
    max_i = size[0]
    max_j = size[1]
    MAP2 = [[" . " for i in range(max_j)] for j in range(max_i)]

    for j in range(0, max_j):
        for i in range(0, max_i):
            if MAP[i][j] > 0:
                MAP2[i][j] = "111"

   # print("x_spacing is " + str(x_spacing))
   # print("y_spacing is " + str(y_spacing))
    path = dijkstras(pos_init,pos_goal)
    print("=====================================")
    print(path)

if __name__ == '__main__':
    main()

