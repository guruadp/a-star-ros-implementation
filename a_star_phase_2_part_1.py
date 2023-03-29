import numpy as np
from sortedcollections import OrderedSet
import math

obstacle_points = []

# size of the map
X_SIZE = 600
Y_SIZE = 250

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

    rpm1 = int(input("Enter RPM1: "))
    rpm2 = int(input("Enter RPM2: "))

    clearance = int(input("Enter clearance: "))
    return start_node, goal_node, obstacle_points, hexagon_pts, triangle_pts, rpm1, rpm2, clearance

start_node, goal_node, obstacle_points, hexagon_pts, triangle_pts, rpm1, rpm2, clearance = get_input()