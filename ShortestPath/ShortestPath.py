#!/usr/bin/python

import sys
import numpy as np
import yaml
from queue import PriorityQueue

MAX_FLOAT = sys.float_info.max
MAP = {}
MAP2 = {}
X_SPACING = 1
Y_SPACING = 1

class Node:
    # dist is distance to start node
    # point is a 2-elem tuple
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
        print("Parent " + self.parent.to_string())
        print("---")

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
    MAP2[i][j] = symbol

def print_map2():  
    size = np.shape(MAP2)
    max_i = size[0]
    max_j = size[1]
    for j in range(0, max_j):
        for i in range(0, max_i):
            print(MAP2[i][j], end=' ')
        print("")

# return true if map square is available, false otherwise
def grid_square_fair(ij):
    i = ij[0]
    j = ij[1]
    size = np.shape(MAP)
    max_i = size[0]
    max_j = size[1]
    if i < 0 or j < 0  or  i > max_i or j > max_j:
        return False

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

def find_nearest_grid(xy):
    ij = real_to_grid(xy)

    near_i = round(ij[0][0])
    near_j = round(ij[1][0])
    return (near_i, near_j)

def print_path(start_node, goal_node):
    curr_node = goal_node
    while not curr_node == start_node:
        curr_node.print_everything()
        curr_node = curr_node.parent
        input("Press Enter to continue...")

def evaluate_this_neighbor(node_q, node_dict, current_node, neighbor_coord, curr_dist):

    if not grid_square_fair(neighbor_coord):
        return node_q
    neighbor_node = node_dict[neighbor_coord]

    # update its distance and parent
    neighbor_node.parent = current_node
    print("The parent of " + neighbor_node.to_string() + " is  "+ current_node.to_string())
    neighbor_node.distance = curr_dist+1

  #  neighbor_node.print_me_grid()
    
    # add it to the priority queue with new distance
    node_q.put((curr_dist+1, neighbor_coord))
    # return the modified queue
    return node_q

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
    ## Find the nearest node to the start node
    start_coord = find_nearest_grid(start)

    ## Find the nearest node to the end node
    goal_coord = find_nearest_grid(goal)

    print("start is near to " + str(start_coord[0]) + ", " + str(start_coord[1]))
    print("goal is near to " + str(goal_coord[0]) + ", " + str(goal_coord[1]))

    # checking_start = grid_to_real(st, x_spacing, y_spacing)
    # checking_goal = grid_to_real(gl, x_spacing, y_spacing)

    # print("checking_start: " + str(checking_start[0]) + ", " + str(checking_start[1]))
    # print("checking_goal: " + str(checking_goal[0]) + ", " + str(checking_goal[1]))

    # dictionary with key coordinate tuple and value Node
    node_dict = {}
    # premake all the dictionary entries
    size = np.shape(MAP)
    max_i = size[0]
    max_j = size[1]
    for j in range(0, max_j):
        for i in range(0, max_i):
            node_dict[(i,j)] = Node((i,j))

    start_node = node_dict[start_coord]
    start_node.distance = 0
    start_node.print_me_grid()
    goal_node = node_dict[goal_coord]

    # priority queue linking distance from start to tuple with coordinates
    node_q = PriorityQueue()
    node_q.put((0, start_coord))
   # node_dict[goal_coord] = goal_node

    best_q_entry = node_q.get()
    curr_coords = best_q_entry[1]
    print(f"THE START is {curr_coords}")

    mark_on_map(start_coord, 's')
    mark_on_map(goal_coord, 'g')
    print_map2()
    
    while(curr_coords != goal_coord):
        # Take the node with the smallest distance
        curr_i = curr_coords[0]
        curr_j = curr_coords[1]
        current_node = node_dict[curr_coords]
        
        current_node_dist = current_node.distance
       # print("current_node_dist = " + str(current_node_dist))
        
        # find the neighbors of current_node
        north_coord = (curr_i-1, curr_j)
        south_coord = (curr_i+1, curr_j)
        east_coord = (curr_i, curr_j-1)
        west_coord = (curr_i, curr_j+1)

        # Visit all of its neighbors and add them with the distance to the q
        node_q = evaluate_this_neighbor(node_q, node_dict, current_node, north_coord,current_node_dist)
        node_q = evaluate_this_neighbor(node_q, node_dict, current_node, south_coord,current_node_dist)
        node_q = evaluate_this_neighbor(node_q, node_dict, current_node, east_coord, current_node_dist)
        node_q = evaluate_this_neighbor(node_q, node_dict, current_node, west_coord, current_node_dist)

        input("Press Enter to continue...")

        best_q_entry = node_q.get()
        curr_coords = best_q_entry[1]

    print("I found a path!!!!!!!!!!!!!!!!")

    # finish the last node
    print(f"reached goal {goal_coord}")
    goal_node = node_dict[goal_coord]

    print_path(start_node, goal_node)
   # goal_node = node_dict[goal_coord]
  #  goal_node.print_me_grid
   # print("distance is ")

      #  input("Press Enter to continue...")



def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
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
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
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
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
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
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)


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
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 2 length:")
    print(s)



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
    MAP2 = [["A"]*max_i]*max_j
   # print("x_spacing is " + str(x_spacing))
   # print("y_spacing is " + str(y_spacing))
    path = dijkstras(pos_init,pos_goal)
    print(path)

if __name__ == '__main__':
    main()

