import numpy as np
from sortedcollections import OrderedSet
import math
import matplotlib.pyplot as plt
from queue import PriorityQueue
import time
import pygame as pyg

obstacle_points = []
plot_shortest = {}
robot_velocity_tracking = {}
# size of the map
X_SIZE = 600
Y_SIZE = 250


# Radius of the Turtlebot3 wheel
r = 0.038
# Length of the Turtlebot3 (Distance between wheels: Wheel base)
L = 0.16

# goal threshold distance from robot location
goal_threshold = 2.5

# Converting rpm to velocity (cm/s)
def convert_vel(wheel_rpm):
    ang_velocity = wheel_rpm*2*math.pi/60
    return ang_velocity

def round_nearest(val):
    if val-int(val)<0.25:
        val=int(val)
    elif val-int(val) >= 0.25 and val-int(val) < 0.75:
        val=int(val)+0.5
    else:
        val = int(val)+1
    
    return val

def euclidean_distance(pt1, pt2):
    distance = math.sqrt(pow((pt1[0] - pt2[0]), 2) + pow((pt1[1] - pt2[1]),2))
    return round(distance, 2)

def find_intersection_pt(a1, a2, intercept1, intercept2, b1, b2):
    A = np.array([[a1, b1], [a2, b2]])
    B = np.array([[intercept1], [intercept2]])
    X = np.linalg.solve(A, B)
    return (X[0][0], X[1][0])

def flip_object_points(points, height, object_height):
    return (points[0], height - points[1] - object_height)

# create map with obstacle
def create_obstacle_map(clearance, points_rate):
    X_SIZE = 6
    Y_SIZE = 2
    plt.xlim([0, X_SIZE])
    plt.ylim([0, Y_SIZE])
    points = OrderedSet()

    for xp in np.arange(0, X_SIZE + points_rate, points_rate):
        for yp in np.arange(0, Y_SIZE + points_rate, points_rate):
            # Border
            if xp <= clearance or xp >= X_SIZE-clearance or yp <= clearance or yp >= Y_SIZE-clearance:
                points.add((round(xp,1), round(yp, 1)))   
                plt.scatter(xp, yp)

            # Rectangle 1
            if yp >= 0.75 - clearance and xp >= 1.50 - clearance and xp <= 1.65 + clearance:
                points.add((round(xp,1), round(yp, 1)))   
                plt.scatter(xp, yp)

            # Rectangle 2
            if yp <= 1.25 + clearance and xp >= 2.50 - clearance and xp <= 2.65 + clearance:
                points.add((round(xp,1), round(yp, 1)))  
                plt.scatter(xp, yp)
            
            # Circle
            if pow((xp - 4.00), 2) + pow((yp - 1.10), 2) - pow((0.5 + 2 * clearance), 2) <= 0:
                points.add((xp,yp))
                plt.scatter(xp, yp)

    plt.show()
    return points

def get_input():
    # Create Obstacle based on the clearance and radius of the robot
    clearance = float(input("Enter clearance: "))
    obstacle_points = create_obstacle_map(clearance, points_rate=0.1)

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

    return start_node, goal_node, obstacle_points, hexagon_pts, triangle_pts, rpm1, rpm2, clearance

def update_theta(angle):
    if angle < 0:
        angle +=360
    elif angle > 360:
        angle = angle%360
        
def move(x_initial,y_initial,theta,ul,ur):
    print("Move function")
    global goal_reached
    t = 0
    #r = 0.038
    r = 0.038
    L = 0.16
    dt = 0.1
    theta_initial = 3.14 * theta / 180
    thetan = theta_initial
    d = 0
    next_x = x_initial
    next_y = y_initial
    intermediate_points = []
    intermediate_points.append((x_initial,y_initial))
    while t<1:
        t = t + dt
        dx = 0.5*r * (ul + ur) * math.cos(thetan) * dt
        dy = 0.5*r * (ul + ur) * math.sin(thetan) * dt
        thetan += (r / L) * (ur - ul) * dt
        next_x = next_x + dx
        next_y = next_y + dy
        #if (round_nearest(next_x), round_nearest(next_y)) not in obstacle_points:
        if (np.round(next_x,1), np.round(next_y,1)) not in obstacle_points:
            intermediate_points.append((next_x, next_y))
            d = d + math.sqrt(math.pow(dx,2)+math.pow(dy ,2))
        else:
            break
    
    print(next_x,next_y)
    if visited_nodes[int(next_x*2)][int(next_y*2)]!= 1 and (np.round(next_x,1), np.round(next_y,1)) not in obstacle_points:
        print("entered if condition inside the move function")
        x_v = (next_x-x_initial)/t
        y_v = (next_y-y_initial)/t
        theta_v = (thetan - theta_initial)/t
        thetan = update_theta(thetan)
        thetan = int(thetan*180/3.14)
        next_point = (next_x, next_y, thetan)
        robot_velocity_tracking[(next_point,(x_initial,y_initial,theta))] = (x_v,y_v,theta_v)
        # Intermediate points are the co-ordinates (path of the robot) from prev_x,prev_y to x_next,y_next 
        intermediate_points.append((next_x, next_y))
        #print("The next point is :", next_point)
        cost_to_come = d
        cost_to_go = euclidean_distance((next_x,next_y),goal_pt)
        total_cost = cost_to_come+cost_to_go
        next_node = (total_cost, cost_to_go, cost_to_come, next_point)

        for i in range(map_queue.qsize()):
            if map_queue.queue[i][3][0] == next_point[0] and map_queue.queue[i][3][1] == next_point[1]:
            #if euclidean_distance(next_point,map_queue.queue[i][2]) <= 0.5 and (next_angle - map_queue.queue[i][2][2]) <= 30:
                if map_queue.queue[i][0] > total_cost:
                    map_queue.queue[i] = next_node 
                    parent_child_info[next_point] = (x_initial,y_initial,theta)
                    plot_shortest[(next_point,(x_initial,y_initial,theta))] = intermediate_points
                    return 
                else:
                    return
        map_queue.put(next_node)
        parent_child_info[next_point] = (x_initial,y_initial,theta)
        # plot_shortest is a dictionary with:
        # Key : (child,parent)
        # Value : List of intermediate points used to plot the trajectory of robot from x_initial,y_initial to x_next,y_next 
        plot_shortest[(next_point,(x_initial,y_initial,theta))] = intermediate_points
        visited_pts.append(next_point)
        if cost_to_go < goal_threshold:
            goal_reached = True
        else:
            goal_reached = False

    #return d, intermediate_points

def back_tracking(path, initial_state, curr_val, plot_shortest,robot_velocity_tracking):
    robot_velocity = []
    coords = []
    optimal_path = []
    optimal_path.append(curr_val)
    parent_path = (curr_val)
   # child = path[parent_path]
    while parent_path != initial_state:  
        coords.append((plot_shortest[(parent_path,path[parent_path])]))
        ## robot_velocity is a list of tuples where each tuple is the x,y,z velocities, which needs to be published every second to the cmd_vel topic
        robot_velocity.append((robot_velocity_tracking[(parent_path,path[parent_path])]))
        parent_path = path[parent_path]
        optimal_path.append(parent_path)
    """ while child != initial_state:  
        child = path[parent_path]
        optimal_path.append(child)
        coords.append((plot_shortest[(child,path[curr_val])])) """
    
    optimal_path.reverse()
    coords.reverse()
    return optimal_path,coords,robot_velocity

def pygame_visualization(clearance):
    pyg.init()
    window = pyg.display.set_mode((X_SIZE,Y_SIZE))

    obstacle_color = "red"
    clearance_color = "pink"
    condition = True
    clock = pyg.time.Clock()

    rect1_clearance = flip_object_points([150 - clearance, 75 - clearance], Y_SIZE, 125+clearance)
    rect2_clearance = flip_object_points([250 - clearance, 0], Y_SIZE, 125+clearance)

    rect1_original = flip_object_points([150, 75], Y_SIZE, 125)
    rect2_original = flip_object_points([250, 0], Y_SIZE, 125)

    # circle_clearance = flip_points([400, 110], Y_SIZE)
    # circle_original = flip_points([400, 110], Y_SIZE)

    while condition:
        for loop in pyg.event.get():
            if loop.type == pyg.QUIT:
                condition = False

        pyg.draw.rect(window, clearance_color ,pyg.Rect(0, 0, X_SIZE, clearance))
        pyg.draw.rect(window, clearance_color ,pyg.Rect(0, Y_SIZE - clearance, X_SIZE, clearance))
        pyg.draw.rect(window, clearance_color ,pyg.Rect(0, 0, clearance, Y_SIZE))
        pyg.draw.rect(window, clearance_color ,pyg.Rect(X_SIZE - clearance, 0, clearance, Y_SIZE))

        pyg.draw.rect(window, clearance_color, [rect1_clearance[0], rect1_clearance[1], 15 + 2*clearance, 125 + clearance], 0)
        pyg.draw.rect(window, clearance_color, [rect2_clearance[0], rect2_clearance[1], 15 + 2*clearance, 125 + clearance], 0)
        pyg.draw.rect(window, obstacle_color, pyg.Rect(rect2_original[0], rect2_original[1], 15, 125))
        pyg.draw.rect(window, obstacle_color, pyg.Rect(rect1_original[0], rect1_original[1], 15, 125))

        pyg.draw.circle(window, clearance_color, (400, 110), 50 + clearance)
        pyg.draw.circle(window, obstacle_color, (400, 110), 50)

        condition = False
    pyg.display.flip()
    pyg.time.wait(3000)
    pyg.quit()

start_node, goal_node, obstacle_points, hexagon_pts, triangle_pts, rpm1, rpm2, clearance = get_input()
action_set = [[0, rpm1], [rpm1, 0], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]]

start = time.time()
map_queue = PriorityQueue()
start_pt = (start_node[0], start_node[1])
goal_pt = (goal_node[0], goal_node[1])
map_queue.put((euclidean_distance(start_pt, goal_pt), euclidean_distance(start_pt, goal_pt), 0, start_node))

visited_nodes = np.zeros((1200,500), dtype=int)
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

    if visited_nodes[int(x*2)][int(y*2)] != 1:
        print("Visiting new node")
        visited_nodes[int(x*2)][int(y*2)]=1 
        if euclidean_distance((x,y), goal_pt) > goal_threshold:
            for action in action_set:
                if goal_reached == False:
                    move(x,y,theta,convert_vel(action[0]),convert_vel(action[1]))


        else:
            X = []
            Y = []
            print("Reached Goal")
            print("The Targeted goal is : ", goal_pt)
            print("Reached goal is : ",(x,y))
            stop = time.time()
            print("Time: ",stop - start)   
                #shortest = back_tracking(parent_child_info, start_pt, goal_pt)
            shortest, plot_data, robot_velocity_list = back_tracking(parent_child_info, start_node, current_node[3], plot_shortest, robot_velocity_tracking)
            print("The shortest path is :")
            print(shortest)
            print("Robot velocity list")
            print(robot_velocity_list)
            for i in range(0,len(plot_data)):
                for j in range(0,11):
                    x,y = plot_data[i][j]
                    X.append(x)
                    Y.append(y)
                    
            # print("-----")
                # plt.scatter(pt[0], pt[1])
            #plt.plot(X, Y)
            plt.scatter(X,Y)
            plt.show()
            break

pygame_visualization(clearance = 5)