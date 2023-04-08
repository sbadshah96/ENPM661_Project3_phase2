#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import math
import heapdict
import numpy as np
import time
import vidmaker
from sortedcollections import OrderedSet
import pygame


# Calculate new 'C' value for new obstacle definition.
def calculate_new_c(m, c, buffer_val):
    if m > 0 and c < 0:
        c_new = c - ((buffer_val) * np.sqrt(1 + (m ** 2)))
        return c_new
    elif m < 0 and c > 0:
        if c > 300:
            c_new = c + ((buffer_val) * np.sqrt(1 + (m ** 2)))
            return c_new
        elif c < 300:
            c_new = c - ((buffer_val) * np.sqrt(1 + (m ** 2)))
            return c_new
    elif m > 0 and c > 0:
        c_new = c + ((buffer_val) * np.sqrt(1 + (m ** 2)))
        return c_new
    else:
       return None

# Define new obstacles based on user input buffer
def obstacles_rec(obstacle_buffer, robot_size=10.5):
    obstacles = []

    buffer_val = obstacle_buffer + robot_size

    c1_rec1 = 250
    m1_rec1 = 0
    c1_rec1_new = c1_rec1 - buffer_val
    obstacles.append((m1_rec1, c1_rec1_new))

    c2_rec1 = 265
    m2_rec1 = 0
    c2_rec1_new = c2_rec1 + buffer_val
    obstacles.append((m2_rec1, c2_rec1_new))

    c3_rec1 = 125
    m3_rec1 = 0
    c3_rec1_new = c3_rec1 + buffer_val
    obstacles.append((m3_rec1, c3_rec1_new))

    c1_rec2 = 150
    m1_rec2 = 0
    c1_rec2_new = c1_rec2 - buffer_val
    obstacles.append((m1_rec2, c1_rec2_new))

    c2_rec2 = 165
    m2_rec2 = 0
    c2_rec2_new = c2_rec2 + buffer_val
    obstacles.append((m2_rec2, c2_rec2_new))

    c3_rec2 = 75
    m3_rec2 = 0
    c3_rec2_new = c3_rec2 - buffer_val
    obstacles.append((m3_rec2, c3_rec2_new))


    c1_bound = 0 + buffer_val
    c2_bound = 600 - buffer_val
    c3_bound = 0 + buffer_val
    c4_bound = 200 - buffer_val
    obstacles.append((0, c1_bound))
    obstacles.append((0, c2_bound))
    obstacles.append((0, c3_bound))
    obstacles.append((0, c4_bound))

    return obstacles

def obstacles_circ(obstacle_buffer, robot_size=10.5):
    a = 400
    b = 110
    c = 50 + obstacle_buffer + robot_size

    return a,b,c


# Check if the robot is in obstacle space.
def check_obstacles(x, y):
    c1_rec1 = obstacles_var1[0][1]
    c2_rec1 = obstacles_var1[1][1]
    c3_rec1 = obstacles_var1[2][1]

    c1_rec2 = obstacles_var1[3][1]
    c2_rec2 = obstacles_var1[4][1]
    c3_rec2 = obstacles_var1[5][1]

    a1_circ = obstacles_var2[0]
    b1_circ = obstacles_var2[1]
    c1_circ = obstacles_var2[2]

    c1_bound = obstacles_var1[6][1]
    c2_bound = obstacles_var1[7][1]
    c3_bound = obstacles_var1[8][1]
    c4_bound = obstacles_var1[9][1]


    if (((c1_rec1) <= x <= (c2_rec1)) and (0 <= y <= (c3_rec1))):
        return False
    elif (((c1_rec2) <= x <= (c2_rec2)) and ((c3_rec2) <= y <= 200)):
        return False
    elif ((x <= c1_bound) or (x >= c2_bound) or (y <= c3_bound) or (y >= c4_bound)):
        return False
    elif ((x-a1_circ)**2 + (y-b1_circ)**2 <= c1_circ**2):
        return False
    else:
        return True

# Custom rounding off function for angle
def custom_ang_round(b):
    if b >= 360:
        b = b % 360
    elif -360 < b < 0:
        b += 360
    elif b <= -360:
        b = b % 360 + 360
    return b

# Visited nodes threshold
def visited_nodes_threshold_check(x, y, theta):
    if visited_nodes[int(x)][int(y)][int(theta)]:
        return False
    elif visited_nodes[int((x + 1))][int(y)][int(theta)]:
        return False
    elif visited_nodes[int((x - 1))][int(y)][int(theta)]:
        return False
    elif visited_nodes[int((x))][int((y + 1))][int(theta)]:
        return False
    elif visited_nodes[int((x))][int((y - 1))][int(theta)]:
        return False
    elif visited_nodes[int((x + 1))][int((y + 1))][int(theta)]:
        return False
    elif visited_nodes[int((x - 1))][int((y + 1))][int(theta)]:
        return False
    elif visited_nodes[int((x + 1))][int((y - 1))][int(theta)]:
        return False
    elif visited_nodes[int((x - 1))][int((y - 1))][int(theta)]:
        return False
    

    if visited_nodes[int(x)][int(y)][int(theta)]:
        return False
    elif visited_nodes[int((x + 2))][int(y)][int(theta)]:
        return False
    elif visited_nodes[int((x - 2))][int(y)][int(theta)]:
        return False
    elif visited_nodes[int((x))][int((y + 2))][int(theta)]:
        return False
    elif visited_nodes[int((x))][int((y - 2))][int(theta)]:
        return False
    elif visited_nodes[int((x + 2))][int((y + 2))][int(theta)]:
        return False
    elif visited_nodes[int((x - 2))][int((y + 2))][int(theta)]:
        return False
    elif visited_nodes[int((x + 2))][int((y - 2))][int(theta)]:
        return False
    elif visited_nodes[int((x - 2))][int((y - 2))][int(theta)]:
        return False


    elif visited_nodes[int((x + 3))][int(y)][int(theta)]:
        return False
    elif visited_nodes[int((x - 3))][int(y)][int(theta)]:
        return False
    elif visited_nodes[int((x))][int((y + 3))][int(theta)]:
        return False
    elif visited_nodes[int((x))][int((y - 3))][int(theta)]:
        return False
    elif visited_nodes[int((x + 3))][int((y + 3))][int(theta)]:
        return False
    elif visited_nodes[int((x - 3))][int((y + 3))][int(theta)]:
        return False
    elif visited_nodes[int((x + 3))][int((y - 3))][int(theta)]:
        return False
    elif visited_nodes[int((x - 3))][int((y - 3))][int(theta)]:
        return False
    else:
        return True

# Check new node based on action set and making decisions to adding it to visited nodes list 
def check_new_node(x, y, theta, total_cost, cost_to_go, cost_to_come,interim_points,interim_velocity):
    x = np.round(x,1)
    y = np.round(y,1)

    theta = custom_ang_round(np.round(theta,2))
    if visited_nodes_threshold_check(x, y, theta):
        if visited_nodes[int(x)][int(y)][int(theta)] == 0:
            if (x, y, theta) in explored_nodes:
                if explored_nodes[(x, y, theta)][0] >= total_cost:
                    explored_nodes[(x, y, theta)] = total_cost, cost_to_go, cost_to_come
                    node_records[(x, y, theta)] = (pop[0][0], pop[0][1], pop[0][2]), interim_points
                    visited_nodes_track.add((x, y, theta))
                    velocity_track[(x, y, theta)] = (pop[0][0], pop[0][1], pop[0][2]), interim_velocity
                    return None
                else:
                    return None
            explored_nodes[(x, y, theta)] = total_cost, cost_to_go, cost_to_come
            node_records[(x, y, theta)] = (pop[0][0], pop[0][1], pop[0][2]), interim_points
            explored_mapping.append((x, y))
            visited_nodes_track.add((x, y, theta))
            velocity_track[(x, y, theta)] = (pop[0][0], pop[0][1], pop[0][2]), interim_velocity

# Non-holonomic constraint function
def action(RPM_L,RPM_R,pop):
    t = 0
    dt = 0.25
    R = 3.3
    L = 17.8
    x = pop[0][0]
    y = pop[0][1]

    theta = pop[0][2]
    interim_points = OrderedSet()
    interim_velocity = []

    x_new = x
    y_new = y
    theta_new = np.deg2rad(theta)
    
    interim_points.add((to_pygame((x_new,y_new),200)))

    while t < 1:
        theta_new += (R/L)*(RPM_R-RPM_L)*dt*2*math.pi/60
        x_new += ((R/2)*(RPM_L+RPM_R)*np.cos((theta_new))*dt*2*math.pi/60)
        y_new += ((R/2)*(RPM_L+RPM_R)*np.sin((theta_new))*dt*2*math.pi/60)

        temp_obs = check_obstacles(x_new,y_new)
        if not temp_obs:
            break
        interim_points.add((to_pygame((x_new,y_new),200)))
        t = t + dt

    Ul = RPM_L*2*math.pi/60
    Ur = RPM_R*2*math.pi/60
    xd_new = (R/2)*(Ul+Ur)*np.cos(theta_new)
    yd_new = (R/2)*(Ul+Ur)*np.sin(theta_new)
    thetad_new = ((R/L)*(Ur-Ul))
    interim_velocity.append((float(xd_new/100),float(yd_new/100),float(thetad_new)))
    
    obs = check_obstacles(x_new, y_new)
    
    if obs:
        new_cost_to_go =  1.75 * np.sqrt(((x_new - x_f) ** 2) + ((y_new - y_f) ** 2))
        new_cost_to_come = np.sqrt(((x_new - x_s) ** 2) + ((y_new - y_s) ** 2))
        new_total_cost = new_cost_to_go + new_cost_to_come
        check_new_node(x_new, y_new, np.rad2deg(theta_new), new_total_cost, new_cost_to_go, new_cost_to_come,interim_points,interim_velocity)

# Backtrack to find the optimal path
def backtracking(x, y, theta):
    backtrack.append((x, y, theta))
    key = node_records[(x, y, theta)][0]
    backtrack.append(key)
    while key != init_pos:
        key = node_records[key][0]
        backtrack.append(key)
    return backtrack[::-1]

def vel_backtracking(x, y, theta):
    vel_backtrack.append((0,0,0))
    key = velocity_track[(x, y, theta)]
    # print('Vel key1: ',key)
    vel_backtrack.extend(key[1])
    while key[0] != init_pos:
        key = velocity_track[key[0]]
        # print('Vel key2: ',key)
        vel_backtrack.extend(key[1])
    return vel_backtrack[::-1]

# Find intersecting coordinates based on (m,c) values
def find_intersection(m1, m2, c1, c2, a, b):
    A = np.array([[-m1, a], [-m2, b]])
    B = np.array([c1, c2])
    X = np.linalg.solve(A, B)
    return X

# Convert coordinates into pygame coordinates
def to_pygame(coords, height):
    return coords[0], height - coords[1]

# Convert an object's coordinates into pygame coordinates
def rec_pygame(coords, height, obj_height):
    return coords[0], height - coords[1] - obj_height


RPM1 = 20
RPM2 = 30

obstacle_buffer = 5

obstacles_var1 = obstacles_rec(obstacle_buffer)
obstacles_var2 = obstacles_circ(obstacle_buffer)
py_obstacles = obstacles_rec(obstacle_buffer,10.5)


x_s = 50
y_s = 100

theta_s = 0
init_pos = (x_s,y_s,theta_s)


x_f = 550
y_f = 100
goal_pos = (x_f,y_f)


# Global variable initialization 
explored_nodes = heapdict.heapdict()
explored_mapping = []
visited_nodes = np.zeros((600, 200, 360))
visited_nodes_track = OrderedSet()
backtrack = []
vel_backtrack = []
node_records = {}
velocity_track = {}
pop = []
the_path = []
index = 0


def move(vel_path):
    # Initializing a new node
    rospy.init_node('a_star', anonymous=False)
    
    #Creating a publisher that publishes velocity commands to /cmd_vel topic
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    #Creating message of type Twist()
    vel_msg = Twist()

    print("Moving Robot with A-star path!")
    rate = rospy.Rate(4.5)
    
    for i in vel_path:
        start = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - start) < 1:
            vel_msg.linear.x = math.sqrt(i[0]**2 + i[1]**2)
            vel_msg.angular.z = i[2]
            print("x_dot, y_dot, theta_dot: ", i)
            vel_pub.publish(vel_msg)
            rate.sleep()

####Pygame Visualization####
def viz():
    pygame.init()
    size = [600, 200]
    d = obstacle_buffer + 10.5
    monitor = pygame.display.set_mode(size)
    pygame.display.set_caption("Arena")

    Done = False
    clock = pygame.time.Clock()
    while not Done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                Done = True
        monitor.fill("black")

        # Walls
        pygame.draw.rect(monitor, "red", [0, 0, d, 200], 0)
        pygame.draw.rect(monitor, "red", [0, 0, 600, d], 0)
        pygame.draw.rect(monitor, "red", [0, 200-d, 600, d], 0)
        pygame.draw.rect(monitor, "red", [600-d, 0, d, 200], 0)

        # Rectangles
        x, y = rec_pygame([250-d, 0], 200, 125+d)
        pygame.draw.rect(monitor, "red", [x, y, 15+2*d, 125+d], 0)

        x, y = rec_pygame([150-d, 75-d], 200, 125+d)
        pygame.draw.rect(monitor, "red", [x, y, 15+2*d, 125+d], 0)

        x, y = rec_pygame([250, 0], 200, 125)
        pygame.draw.rect(monitor, "orange", [x, y, 15, 125], 0)

        x, y = rec_pygame([150, 75], 200, 125)
        pygame.draw.rect(monitor, "orange", [x, y, 15, 125], 0)

        # Circle
        pygame.draw.circle(monitor, "red", to_pygame((400,110), 200), radius=50+d)
        pygame.draw.circle(monitor, "orange", to_pygame((400,110), 200), radius=50)

        for i in the_path:
            pygame.draw.circle(monitor, (0, 255, 0), to_pygame(i, 200), 2)
            pygame.display.flip()
            clock.tick(20)

        pygame.display.flip()
        pygame.time.wait(1000)
        Done = True

    pygame.quit()



# The A* algorithm
if __name__ == '__main__':
    start = time.time()

    if check_obstacles(x_s, y_s) and check_obstacles(x_f, y_f):
        print('A-starring........')
        init_cost_to_go = round(np.sqrt(((x_s - x_f) ** 2) + ((y_s - y_f) ** 2)), 1)
        init_cost_to_come = 0
        init_total_cost = init_cost_to_come + init_cost_to_go
        explored_nodes[(x_s, y_s, theta_s)] = init_total_cost, init_cost_to_go, init_cost_to_come
        explored_mapping.append((x_s, y_s))
        while len(explored_nodes):
            pop = explored_nodes.popitem()
            index += 1
            if not (x_f - 2 < pop[0][0] < x_f + 2 and y_f - 2 < pop[0][1] < y_f + 2):
                if visited_nodes[int(pop[0][0])][int(pop[0][1])][int(pop[0][2])] == 0:
                    visited_nodes[int(pop[0][0])][int(pop[0][1])][int(pop[0][2])] = 1

                    action(0,RPM1,pop)

                    action(RPM1,0,pop)

                    action(RPM1,RPM1,pop)

                    action(0,RPM2,pop)

                    action(RPM2,0,pop)

                    action(RPM2,RPM2,pop)

                    action(RPM1,RPM2,pop)

                    action(RPM2,RPM1,pop)


            else:
                print('Goal Reached!')
                print('Explored Nodes: ',list(explored_nodes.keys()))
                print('Last Pop: ', pop)
                the_path = backtracking(pop[0][0], pop[0][1], pop[0][2])
                print('Backtracking: ', the_path)
                the_vel_path = vel_backtracking(pop[0][0], pop[0][1], pop[0][2])
                # print('Vel Backtracking: ', the_vel_path)
                end = time.time()
                print('Time: ', round((end - start), 2), 's')
                print('Iterations: ',index)

                viz()
                move(the_vel_path)
                break

        if not len(explored_nodes):
            print('No solution found.')
            print('Explored Nodes: ',list(explored_nodes.keys()))
            print('Last Pop: ', pop)
            end = time.time()
            print('Time: ', round((end - start), 2), 's')
            print('Iterations: ',index)

    elif not check_obstacles(x_s, y_s):
        print('Cannot A-star, starting node in an obstacle space.')
    elif not check_obstacles(x_f, y_f):
        print('Cannot A-star, goal node in an obstacle space.')
        
