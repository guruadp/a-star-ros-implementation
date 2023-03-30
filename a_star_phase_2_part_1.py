import numpy as np
from sortedcollections import OrderedSet
import math
import matplotlib.pyplot as plt
from queue import PriorityQueue
import time

obstacle_points = []

# size of the map
X_SIZE = 600
Y_SIZE = 250

def euclidean_distance(pt1, pt2):
    distance = math.sqrt(pow((pt1[0] - pt2[0]), 2) + pow((pt1[1] - pt2[1]),2))
    return round(distance, 2)

def find_intersection_pt(a1, a2, intercept1, intercept2, b1, b2):
    A = np.array([[a1, b1], [a2, b2]])
    B = np.array([[intercept1], [intercept2]])
    X = np.linalg.solve(A, B)
    return (X[0][0], X[1][0])

# create map with obstacle
def create_obstacle_map(clearance):
    points = OrderedSet()
    x_range = np.arange(0, X_SIZE+1, 0.5)
    y_range1 = np.arange(0, clearance+1, 0.5)
    y_range2 = np.arange(Y_SIZE - clearance, Y_SIZE + 1, 0.5)
    for xp in x_range:
        for yp in y_range1:
            points.add((xp,yp))
        for yp in y_range2:
            points.add((xp,yp))

    y_range = np.arange(0, Y_SIZE + 1, 0.5)
    x_range1 = np.arange(0, clearance+1, 0.5)
    x_range2 = np.arange(X_SIZE - clearance, X_SIZE + 1, 0.5)
    for yp in y_range:
        for xp in x_range1:
            points.add((xp,yp))
        for xp in x_range2:
            points.add((xp,yp))

    # Rectangles and clearance
    x_range = np.arange(100 - clearance, 150 + clearance + 1, 0.5)
    y_range = np.arange(0, Y_SIZE+1, 0.5)
    for xp in x_range:
        for yp in y_range:
            if yp <= (100 + clearance) or yp >= (150 - clearance):
                points.add((xp,yp))

    # triangle 
    x_range = np.arange(460-clearance, 510+2*clearance, 0.5)
    y_range = np.arange(0, Y_SIZE, 0.5)

    m = 2
    b1, b2 = -895, 1145
    
    c1 = b1 + clearance * (math.sqrt(pow(m,2) + 1))
    c2 = b1 - clearance * (math.sqrt(pow(m,2) + 1))
    c3 = b2 + clearance * (math.sqrt(pow(m,2) + 1))
    c4 = b2 - clearance * (math.sqrt(pow(m,2) + 1))

    for xp in x_range:
        for yp in y_range:            
            if (-m*xp+yp >= min(c1, c2)) and (m*xp+yp <= max(c3, c4)):
                    points.add((xp,yp)) 

    triangle_p1 = [460 - clearance, 225 + clearance]
    triangle_p2 = find_intersection_pt(m, 0, max(c3, c4), 225 + clearance, 1, 1)
    triangle_p3 = find_intersection_pt(-m, m, min(c1, c2), max(c3, c4), 1, 1)
    triangle_p4 = find_intersection_pt(-m, 0, min(c1, c2), 25 - clearance, 1, 1)
    triangle_p5 = [460 - clearance, 25 - clearance]

    triangle_pts = [triangle_p1, triangle_p2, triangle_p3, triangle_p4, triangle_p5]

    # Hexagon
    x_range = np.arange(300 - int(64.95) - clearance, 300 + int(64.95) + clearance, 0.5)
    y_range = np.arange(125 - 75 - clearance, 125 + 75 + clearance, 0.5)

    m = 15/26
    b1, b2, b3, b4 = 26.92, 373.07, 223.07, -123.21
    
    c1 = b1 + clearance * (math.sqrt(pow(m,2) + 1))
    c2 = b1 - clearance * (math.sqrt(pow(m,2) + 1))
    c3 = b2 + clearance * (math.sqrt(pow(m,2) + 1))
    c4 = b2 - clearance * (math.sqrt(pow(m,2) + 1))
    c5 = b3 + clearance * (math.sqrt(pow(m,2) + 1))
    c6 = b3 - clearance * (math.sqrt(pow(m,2) + 1))
    c7 = b4 + clearance * (math.sqrt(pow(m,2) + 1))
    c8 = b4 - clearance * (math.sqrt(pow(m,2) + 1))

    for xp in x_range:
        for yp in y_range:            
            if  (yp - m*xp - max(c1, c2)) <= 0 and (yp + m*xp - max(c3, c4)) <= 0 and (yp - m*xp - min(c7, c8)) >= 0 and (yp + m*xp + min(c5, c6)) >= 0:
                points.add((xp,yp))    

    hexagon_p1 = find_intersection_pt(-m, m, max(c1, c2), max(c3, c4), 1, 1)
    hexagon_p2 = find_intersection_pt(m, 1, max(c3, c4), 300 + 64.95 + clearance, 1, 0)
    hexagon_p3 = find_intersection_pt(1, -m, 300 + 64.95 + clearance, min(c7, c8), 0, 1)
    hexagon_p4 = find_intersection_pt(m, -m, min(c5, c6), min(c7, c8), 1, 1)
    hexagon_p5 = find_intersection_pt(m, 1, min(c5, c6), 300 - 64.95 -clearance, 1, 0)
    hexagon_p6 = find_intersection_pt(1, -m, 300 - 64.95-clearance, max(c1, c2), 0, 1)

    hexagon_pts = [hexagon_p1, hexagon_p2, hexagon_p3, hexagon_p4, hexagon_p5, hexagon_p6]

    return points, hexagon_pts, triangle_pts

def get_input():
    # Create Obstacle based on the clearance and radius of the robot
    obstacle_points, hexagon_pts, triangle_pts = create_obstacle_map(5)

    accept_start_node, accept_goal_node = True, True
    while accept_start_node:
        start_x = int(input("Enter start x: "))
        start_y = int(input("Enter start y: "))
        start_theta = int(input("Enter start orientation: "))
        start_node = (start_x, start_y, start_theta)

        if start_node not in obstacle_points and start_x < X_SIZE and start_y < Y_SIZE:
            accept_start_node = False
        else:
            print("Entered start node is either an obstacle or out of map. Please enter a valid co-ordinate...")

    while accept_goal_node:    
        goal_x = int(input("Enter goal x: "))
        goal_y = int(input("Enter goal y: "))        
        goal_node = (goal_x, goal_y)

        if goal_node not in obstacle_points and goal_x < X_SIZE and goal_y < Y_SIZE:
            accept_goal_node = False
        else:
            print("Entered goal node is either an obstacle or out of map. Please enter another co-ordinate...")    

    rpm1 = int(input("Enter rpm1 : "))
    rpm2 = int(input("Enter rpm2 : "))

    clearance = int(input("Enter clearance: "))
    return start_node, goal_node, obstacle_points, hexagon_pts, triangle_pts, rpm1, rpm2, clearance

def move(x_initial,y_initial,theta,ul,ur):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    thetan = 3.14 * theta / 180
    d = 0
    next_x = x_initial
    next_y = y_initial
    intermediate_points = []
    while t<1:
        t = t + dt
        dx = 0.5*r * (ul + ur) * math.cos(thetan) * dt
        dy = 0.5*r * (ul + ur) * math.sin(thetan) * dt
        thetan += (r / L) * (ur - ul) * dt
        next_x = next_x + dx
        next_y = next_y + dy
        if (next_x, next_y) not in obstacle_points:
            intermediate_points.append((next_x, next_y))
            d = d + math.sqrt(math.pow(dx,2)+math.pow(dy ,2))
        else:
            break
    
    #next_point = (next_x, next_y)
    next_point = (next_x, next_y, thetan)
    print("The next point is :", next_point)
    cost_to_come = d
    cost_to_go = euclidean_distance((next_x,next_y),(x_initial,y_initial))
    total_cost = cost_to_come+cost_to_go
    next_node = (total_cost, cost_to_go, cost_to_come, next_point)

    for i in range(map_queue.qsize()):
        if map_queue.queue[i][3] == next_point:
        #if euclidean_distance(next_point,map_queue.queue[i][2]) <= 0.5 and (next_angle - map_queue.queue[i][2][2]) <= 30:
            if map_queue.queue[i][0] > total_cost:
                map_queue.queue[i] = next_node 
                parent_child_info[next_point] = (x_initial,y_initial)
                visited_pts.append(next_point)
                return 
            else:
                return
    map_queue.put(next_node)
    parent_child_info[next_point] = (x_initial,y_initial)
    visited_pts.append(next_point)

    #return d, intermediate_points

start_node, goal_node, obstacle_points, hexagon_pts, triangle_pts, rpm1, rpm2, clearance = get_input()
rpm1 = 15
rpm2 = 25
action_set = [[0, rpm1], [rpm1, 0], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]]

start = time.time()
map_queue = PriorityQueue()
start_pt = (start_node[0], start_node[1])
goal_pt = (goal_node[0], goal_node[1])
map_queue.put((euclidean_distance(start_pt, goal_pt), euclidean_distance(start_pt, goal_pt), 0, start_node))

visited_nodes = np.zeros((1200,500,12), dtype=int)
parent_child_info = {}
shortest_path = []
visited_pts= []
visualization_points = create_obstacle_map(clearance)
goal_reached = False
while map_queue.qsize() != 0:
    current_node = map_queue.get()
    x, y, theta = current_node[3][0], current_node[3][1], current_node[3][2]
    print("The current node inside the while True is : ")
    print(current_node)

    if visited_nodes[int(x*2)][int(y*2)][int(theta/30)] != 1:
        visited_nodes[int(x*2)][int(y*2)][int(theta/30)]=1 
        if euclidean_distance((x,y), goal_pt) > 4.5:
            #for action in action_set:
            move(x,y,theta,100,100)


        else:
            print("Reached Goal")
            stop = time.time()
            print("Time: ",stop - start)   
                #shortest = back_tracking(parent_child_info, start_pt, goal_pt)
            """ shortest = back_tracking(parent_child_info, start_node, current_node[3])
                #shortest.reverse()  
                print(shortest) """
            break

# xs = (np.where(visited_nodes == 1)[0])/2
# ys = (np.where(visited_nodes == 1)[1])/2
# visited_pts = []
# for x, y in zip(xs, ys):
#     visited_pts.append((int(x), int(y)))
# print(visited_pts)    

""" for action in action_set:
    d, intermediate_points = cost(0, 0, 60, action[0], action[1])
    x = []
    y = []
    for pt in intermediate_points:
        # print(pt)
        x.append(pt[0])
        y.append(pt[1])
    # print(intermediate_points)
    plt.plot(x, y)
    # print("-----")
        # plt.scatter(pt[0], pt[1])
plt.show() """