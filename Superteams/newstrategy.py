from controller import Robot, GPS, Lidar, Camera, InertialUnit #type: ignore
import cv2
import numpy as np
import math
import struct
from collections import deque
import random
#region defines
###############################################################DEFINES######################################################################
#define robot
robot = Robot()
timeStep = 32
timestep = 32


# define motors
right_wheel = robot.getDevice("wheel1 motor") #wheel1
left_wheel = robot.getDevice("wheel2 motor") #wheel2


#reseting motors
right_wheel.setPosition(float('inf'))
left_wheel.setPosition(float('inf'))
right_wheel.setVelocity(0.0)
left_wheel.setVelocity(0.0)


#define sensors
right_camera = robot.getDevice("cameraR")
left_camera = robot.getDevice("cameraL")
compass = robot.getDevice("inertial_unit") #inertial_unit
gps = robot.getDevice("gps")
color = robot.getDevice("colour_sensor")
lidar = robot.getDevice("lidar")
#distance_sensor = robot.getDevice("distance sensor1")
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(timestep)


#enabling sensors
right_camera.enable(timestep)
left_camera.enable(timestep)
compass.enable(timestep)
gps.enable(timestep)
color.enable(timestep)
lidar.enable(timestep)
#distance_sensor.enable(timestep)


#define constants
max_velocity = 6.28
pi = math.pi


#define variables
velocity = max_velocity
compass_value=0
lidar_value=0
gps_readings = [0, 0, 0]


#define victim detection variables (letter)
detected_letter = None 
letter_is_stopped = False 
letter_stop_start_time = 0

#endregion

#region sensor values

#lidar 
def get_lidar_value():
    global lidar_value
    lidar_value = []
    range_image = lidar.getRangeImage()
    for i in range(4):
        lidar_value.append([])
        for j in range(512):
            lidar_value[i].append(round(range_image[i * 512 + j] * 100, 2)) #lidar[4][512]


#compass
def get_compass_value():
    global compass_value
    compass_value = compass.getRollPitchYaw()[2]
    compass_value = compass_value * 180 / math.pi  # convert to degrees
    compass_value = round(compass_value, 1)


#gps in cm
def get_gps_readings():
    gps_readings[0] = gps.getValues()[0]*100
    gps_readings[1] = gps.getValues()[1]*100
    gps_readings[2] = gps.getValues()[2]*100


#endregion
###########################################################color sensor#######################################################################
#region color sensor


# example 3shan nstdkhm da
# color_res = color_sensor_detector()
# if color_res[0]:
#     current_area_number = get_area(color_res[1])
# else:
#     //You can handle the swmap, checkpoint, and hole things


# boolean indicating if the robot finished the area or not
a1 = False
a2 = False
a3 = False
a4 = False


current_area_number = 1
mp = {} #stores color el tile dy
color_done={}
area_done={}
#dont call this one
def Color_tile(tile_id, color):
    if color!=0:
        pass
        # print("################################################################################################################################", Node_of_ID[tile_id].x, Node_of_ID[tile_id].y, "color: ", color)
    tile_value = [[color,0,color],[0,0,0],[color,0,color]]
    mp[tile_id] = tile_value
    color_done[color]=True

#byshof kona fen w el robot 2ary anhy color fa byreturn el robot lw dakhal 7yro7 fen
def get_area(color):
    if color==2:                            #red
        if current_area_number==3:
            return 4
        return 3
    elif color==3:                          #green  
        if current_area_number==1:
            return 4
        return 1
    elif color==4:                          #blue
        if current_area_number==2:
            return 1
        return 2
    elif color==5:                          #purple
        if current_area_number==3:
            return 2
        return 3
    elif color==7:                          #Orange
        if current_area_number==4:
            return 2
        return 4
    elif color==8:                          #Yellow
        if current_area_number==1:
            return 3
        return 1
target_color=0
#bydetect colors el tiles
def color_sensor_detector_and_color(tile_id):
    """
    Gates are the tiles colored to get from area to area
    Checks if the tile color is not a shade of grey.
    Returns True if the color is not grey, otherwise False.
    if True, it will also return 2,3,4 area number, 5 swamp, or 6 hole
    """
    image = color.getImage()

    r = color.imageGetRed(image, 1, 0, 0)
    g = color.imageGetGreen(image, 1, 0, 0)
    b = color.imageGetBlue(image, 1, 0, 0)

    # Grey shades have R, G, and B values close to each other
    if abs(r - g) <= 10 and abs(g - b) <= 10 and abs(r - b) <= 10 and r>=200:
        # print(f"Tile is normal tile: R={r}, G={g}, B={b}")
        Color_tile(tile_id, 0)
        return [False,-1]
    elif abs(r - g) <= 10 and abs(g - b) <= 10 and abs(r - b) <= 10 and r<=50:
        # print(f"Tile is black: R={r}, G={g}, B={b}")
        Color_tile(tile_id, 2)
        return [False,6]
    elif r<=120 and b<=120 and g<=120:
        # print(f"Tile is grey checkpoint: R={r}, G={g}, B={b}")
        Color_tile(tile_id, 4)
        return [False,-1]
    elif r > g and r > b and abs(g-b) <=10 and g<=80 and b<=80 and r>=200:
        # print(f"Tile is red: R={r}, G={g}, B={b}")
        Color_tile(tile_id, 'r')
        return [True,2]
    elif g > r and g > b and abs(r-b) <=10 and r<=80 and b<=80 and g>=200:
        # print(f"Tile is green: R={r}, G={g}, B={b}")
        Color_tile(tile_id, 'g')
        return [True,3]
    elif b > r and b > g and abs(r-g) <=10 and r<=80 and g<=80 and b>=200:
        # print(f"Tile is blue: R={r}, G={g}, B={b}")
        Color_tile(tile_id, 'b')
        return [True,4]
    elif r>=100 and g<=100 and b>=100:
        # print(f"Tile is purple: R={r}, G={g}, B={b}")
        Color_tile(tile_id, 'p')
        return [True,5]
    elif r>=200 and 200<=g<=220 and b<=100:
        # print(f"Tile is Orange: R={r}, G={g}, B={b}")
        Color_tile(tile_id, 'o')
        return [True,7]
    elif r>=200 and g>=220 and b<=100:
        # print(f"Tile is Yellow: R={r}, G={g}, B={b}")
        Color_tile(tile_id, 'y')
        return [True,8]
    elif r>=100 and g>=100 and b<=100:
        # print(f"Tile is swamp: R={r}, G={g}, B={b}")
        Color_tile(tile_id, 3)
        return [False,9]
    else:
        # print(f"Tile is probably a checkpoint: R={r}, G={g}, B={b}")
        Color_tile(tile_id, 4)
        return [False,-1]
    

#endregion
#region node class
################################################################node class###########################################################################
global coords, ID, Node_of_ID, visited, adj, global_bad, list_of_colors
global_bad = False
list_of_colors = []
adj = [[] for _ in range(1000)]  # Adjacency list for the graph
bad_nodes = []  # List to store bad nodes
visited = [False] * 1000  # Visited array for the nodes
ID = 1  # Next available ID
Node_of_ID = [None] * 1000  # map to store the nodes by their ID
coords = {} # map to store the id of the coordinates

#id counter
def Get_Next_ID():
    
    global ID
    ID += 1
    return ID - 1

#to know el robot m7tag ybos towards which angle and mover forward in order to move between these 2 tiles
def Get_Direction(node1, node2):
    x1, y1 = node1.x, node1.y
    x2, y2 = node2.x, node2.y

    if abs(x1 - x2) > abs(y1 - y2):
        if x1 < x2:  # move to positive x
            return "2", -90
        else:
            return "1", 90
    else:
        if y1 < y2:
            return "3", 180
        else:
            return "0", 0


class Node:
    def __init__(self, x, y):
        global coords, ID, Node_of_ID, visited, adj
        self.id = Get_Next_ID()
        self.x = x
        self.y = y
        self.area = current_area_number
        if x not in coords:
            coords[x] = {}  # Create a nested dictionary if x doesn't exist
        coords[x][y] = self.id
        Node_of_ID[self.id] = self
        # print("Added node with ID: ", self.id, ", at coordinates: ", x, y)
        visited[self.id] = False
    
    def add_neighbor(self, neighbor):
        global adj, bad_nodes

        if self.id in bad_nodes or neighbor.id in bad_nodes:
            return
        
        adj[self.id].append(neighbor.id)
        adj[neighbor.id].append(self.id)  # Add this line to make it bidirectional
        
    def __str__(self):
        return f"Node({self.x}, {self.y})"
    
    #bfs to find nearest not visited node to this one
    def get_nearest(self):
        global coords, ID, Node_of_ID, visited, adj, list_of_colors
        q = deque()
        vis = [False] * 10000
        q.append([self.id, "", exact_angle(compass_value), []])

        while len(q) > 0:
            x = q.popleft()

            cur_id = x[0]
            cur_directions = x[1]
            cur_angle = x[2]
            list_of_angles = x[3]

            if(vis[cur_id]):
                continue

            vis[cur_id] = True
            cur_node = Node_of_ID[cur_id]

            if not visited[cur_node.id] and cur_node.id not in list_of_colors:
                # print(list_of_angles, "found unvisited node: ", cur_node.id, ", at coordinates: ", cur_node.x, cur_node.y)
                return cur_node, list_of_angles
            
            for i in range(len(adj[cur_node.id])):
                neighbor = Node_of_ID[adj[cur_node.id][i]]
                s, d = Get_Direction(cur_node, neighbor)
                q.append([neighbor.id, cur_directions + s, d, list_of_angles + [d]])

        return None, None
    
    def get_color_tile(self,color):
        global coords, ID, Node_of_ID, visited, adj
        q = deque()
        vis = [False] * 10000
        q.append([self.id, "", exact_angle(compass_value), []])

        while len(q) > 0:
            x = q.popleft()

            cur_id = x[0]
            cur_directions = x[1]
            cur_angle = x[2]
            list_of_angles = x[3]

            if(vis[cur_id]):
                continue

            vis[cur_id] = True
            cur_node = Node_of_ID[cur_id]
            # print("ana bdwr 3l blue",cur_node.x,cur_node.y)
            if cur_node.id in mp:
                pass
                # print("value el MPPPPPP",mp[cur_node.id][0][0])
            if cur_node.id in mp and mp[cur_node.id][0][0]==color:
                # print(list_of_angles, "found colored node: ", cur_node.id, ", at coordinates: ", cur_node.x, cur_node.y)
                return cur_node, list_of_angles
            
            for i in range(len(adj[cur_node.id])):
                neighbor = Node_of_ID[adj[cur_node.id][i]]
                s, d = Get_Direction(cur_node, neighbor)
                q.append([neighbor.id, cur_directions + s, d, list_of_angles + [d]])

        return None, None
    
    def rawa7ony(self):
        global coords, ID, Node_of_ID, visited, adj
        q = deque()
        vis = [False] * 10000
        q.append([self.id, "", exact_angle(compass_value), []])

        while len(q) > 0:
            x = q.popleft()

            cur_id = x[0]
            cur_directions = x[1]
            cur_angle = x[2]
            list_of_angles = x[3]

            if(vis[cur_id]):
                continue

            vis[cur_id] = True
            cur_node = Node_of_ID[cur_id]
            # print("ana bdwr 3l blue",cur_node.x,cur_node.y)
            if cur_node.id in mp:
                pass
                # print("value el MPPPPPP",mp[cur_node.id][0][0])
            if cur_node.id==1:
                # print(list_of_angles, "found colored node: ", cur_node.id, ", at coordinates: ", cur_node.x, cur_node.y)
                return cur_node, list_of_angles
            
            for i in range(len(adj[cur_node.id])):
                neighbor = Node_of_ID[adj[cur_node.id][i]]
                s, d = Get_Direction(cur_node, neighbor)
                q.append([neighbor.id, cur_directions + s, d, list_of_angles + [d]])

        return None, None

#creates a new node in class
def Create_Node(x, y):
    global coords, Node_of_ID, min_x_coord, min_y_coord, max_x_coord, max_y_coord
    min_x_coord= min(min_x_coord, x)
    min_y_coord= min(min_y_coord, y)
    max_x_coord= max(max_x_coord, x)
    max_y_coord= max(max_y_coord, y)
    # print("Wanting to create a node with coordinates: ", x, y, " and ID: ", coords[x][y])
    if(coords[x][y] != 0):
        return Node_of_ID[coords[x][y]]

    return Node(x, y)

#endregion







###############################################################turning###########################################################################

#region turning

#checks angle range and makes it from -180 to 180 only
def check_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


#robot rotates right 90 degrees --> -90
def rotate_right():
    get_compass_value()
    need = (exact_angle(compass_value) - 90) #target angle
    check_angle(need)
    while robot.step(timeStep) != -1:
        get_compass_value()
        error = check_angle(need - compass_value)

        check_if_finished()
    
        # Continuous camera checking during rotation
        Scan_Camera()
        

        if abs(error) < 1.52:
            right_wheel.setVelocity(0)
            left_wheel.setVelocity(0)
            break

        right_wheel.setVelocity(-velocity * 0.29)
        left_wheel.setVelocity(velocity * 0.29)


#robot rotates left 90 degrees --> +90
def rotate_left():
    get_compass_value()
    need = (exact_angle(compass_value) + 90) #target angle
    check_angle(need)
    
    while robot.step(timeStep) != -1:
        get_compass_value()
        error = check_angle(need - compass_value)
        
        # Continuous camera checking during rotation

        check_if_finished()

        Scan_Camera()

            
        
        if abs(error) < 1.52:
            right_wheel.setVelocity(0)
            left_wheel.setVelocity(0)
            break
            
        right_wheel.setVelocity(velocity * 0.29)
        left_wheel.setVelocity(-velocity * 0.29)



# approximates angle to nearest quadrant angle
def exact_angle(angle):
    da=[0,90,180,-180,-90]
    correct_angle=0
    for i in range(5):
        if (abs(angle-da[i])<=5):
            correct_angle=da[i]
            break
    return correct_angle

#if robot is not perfectly towards an axis it fixes it
def adjust_angle():
    get_compass_value()
    # target_angle = round(compass_value / 90) * 90
    target_angle = exact_angle(compass_value)
    error = check_angle(target_angle - compass_value)
    if abs(error) > 3: 
        if error > 0:
            
            right_wheel.setVelocity(velocity * 0.3)
            left_wheel.setVelocity(-velocity * 0.3)
        else:
            
            right_wheel.setVelocity(-velocity * 0.3)
            left_wheel.setVelocity(velocity * 0.3)
        
        
        while robot.step(timestep) != -1:
            get_compass_value()
            error = check_angle(target_angle - compass_value)
            if abs(error) < 1.52:
                stop()
                break

#function to rotate 180 degree
def rotate_180():
    get_compass_value()
    need = (exact_angle(compass_value) + 180)
    need= check_angle(need)
    while robot.step(timeStep) != -1:
        check_if_finished()
        get_compass_value()
        error = check_angle(need - compass_value)
        
        # Continuous camera checking during rotation
        image = right_camera.getImage()
        image2= left_camera.getImage()
        Scan_Camera()

        
        if abs(error) < 1.52:
            right_wheel.setVelocity(0)
            left_wheel.setVelocity(0)
            break
            
        right_wheel.setVelocity(-velocity * 0.29)
        left_wheel.setVelocity(velocity * 0.29)

#function to rotate l7d ma el robot ykon target degrees
def shortest_90(target):
    # print("target angle: ", target)
    get_compass_value()
    curr = exact_angle(compass_value)
    diff = target - curr
    
    diff=check_angle(diff)
        
    direction = 1 if diff > 0 else -1

    while robot.step(timeStep) != -1:
        get_compass_value()
        error = check_angle(target - compass_value)
        

        check_if_finished()
        # Continuous camera checking during rotation

        Scan_Camera()


        
        if abs(error) < 1.52:
            stop()
            break
            
        right_wheel.setVelocity(direction * velocity * 0.29)
        left_wheel.setVelocity(-direction * velocity * 0.29)

    
#endregion
###############################################################moving###########################################################################
#region moving

#required variables here
swamp = False
tile_distance=6
# vis = set()


#both motors positive
def move_forward():
    right_wheel.setVelocity(velocity)
    left_wheel.setVelocity(velocity)


#both motors negative
def move_backward():
    right_wheel.setVelocity(-velocity)
    left_wheel.setVelocity(-velocity)


#both motors zero
def stop():
    right_wheel.setVelocity(0)
    left_wheel.setVelocity(0)


#yshof hwa 7ydetect letter wla hazard
def first (camera):
    firsst = detect_letters(camera)
    if firsst:
        return firsst
    
    return hazard_image_detection(camera)

#used to know the next tile hatkon fen based on his direction
dy = [[-tile_distance, 0, 0, tile_distance],
[0, -tile_distance, tile_distance, 0],
[tile_distance, 0, 0, -tile_distance],
[0, tile_distance, -tile_distance, 0]]
dx = [[0, tile_distance, -tile_distance, 0],
[-tile_distance, 0, 0, tile_distance],
[0, -tile_distance, tile_distance, 0],
[tile_distance, 0, 0, -tile_distance]]
#row indicates its angle, can be retrieved from angle_to idx
#column indicates hwa 3ayz yro7 anhy etgah
#0 forward, 1 right, 2 left, 3 back
da=[0,90,180,-180,-90]
angle_to_idx = {
    0: 0,
    90: 1,
    180: 2,
    -180: 2,
    -90: 3
    }

#lw l2a trap and it is not valid to continue moving in the tile therefore he needs to return to the nearest multiple of 12 ely hwa gh menha
#el x is a parameter bnb3to ll functions which is a boolean indicating el x hwa el bytghyr? yes/no
def move_tile_elfatet(el_x):
    adjust_angle()

    get_gps_readings()
    get_compass_value()
    #calculating el x w el y bto3 el tile el fatt ely hrg3laha
    x=gps_readings[0]-xstart
    x = round(x / 12)
    x*=12
    y=gps_readings[2]-ystart
    y = round(y / 12)
    y*=12
    targetx=x
    targety=y
    
    Scan_Camera()
    adjust_angle()
    move_forward()
    while(robot.step(timestep) != -1):
        colors=color.getImage()
        # print("HERE COLOR",colors)

        check_if_finished()
        
        get_gps_readings()
        adjust_angle()
        Scan_Camera()
        
        if (not el_x) and abs(targety-(gps_readings[2]-ystart)) < 0.25:
            stop()
            break

        if el_x and abs(targetx-(gps_readings[0]-xstart)) < 0.25:
            stop()
            break

    # print("moved 1 odam")
    return 

def move_tile_elfatet_bdahro(el_x,targetx,targety):
    adjust_angle()

    get_gps_readings()
    get_compass_value()

    x, y = get_current_xy()
    cnt = 0
    
    Scan_Camera()
    adjust_angle()
    move_backward()
    
    while(robot.step(timestep) != -1):
        cnt += 1
        colors=color.getImage()

        check_if_finished()
        
        get_gps_readings()
        adjust_angle()
        Scan_Camera()

        if cnt > 5:
            cnt = 0
            x2, y2 = get_current_xy()

            if abs(x2 - x) < 3 and abs(y2 - y) < 3:
                cur_node = Node_of_ID[coords[x][y]]
                node2, list_of_angles = cur_node.get_nearest()

                move_with_angles(list_of_angles)
                break

            x = x2
            y = y2

        
        if (not el_x) and abs(targety-(gps_readings[2]-ystart)) < 0.25:
            stop()
            break

        if el_x and abs(targetx-(gps_readings[0]-xstart)) < 0.25:
            stop()
            break

    return#returns current xy tiles
def get_current_xy():
    x=gps_readings[0]-xstart
    x = round(x / tile_distance)
    x*=tile_distance
    y=gps_readings[2]-ystart
    y = round(y / tile_distance)
    y*=tile_distance
    return x, y

#move one tile forward
# oldx,
def move_tile_forward():
    global coords, ID, Node_of_ID, visited, adj, target_color, tile_distance,dx,dy, global_bad, list_of_colors
    if current_area_number== 1:
        tile_distance=6
        dy = [[-tile_distance, 0, 0, tile_distance],
        [0, -tile_distance, tile_distance, 0],
        [tile_distance, 0, 0, -tile_distance],
        [0, tile_distance, -tile_distance, 0]]
        dx = [[0, tile_distance, -tile_distance, 0],
        [-tile_distance, 0, 0, tile_distance],
        [0, -tile_distance, tile_distance, 0],
        [tile_distance, 0, 0, -tile_distance]]
    else:
        tile_distance=6
        dy = [[-tile_distance, 0, 0, tile_distance],
        [0, -tile_distance, tile_distance, 0],
        [tile_distance, 0, 0, -tile_distance],
        [0, tile_distance, -tile_distance, 0]]
        dx = [[0, tile_distance, -tile_distance, 0],
        [-tile_distance, 0, 0, tile_distance],
        [0, -tile_distance, tile_distance, 0],
        [tile_distance, 0, 0, -tile_distance]]
    
    adjust_angle()

    
    get_gps_readings()
    get_compass_value()

    #calculating el starting tile as a multiple of 12
    x, y = get_current_xy()
    cnt = 0

    addNodes()
    #gps_readings[2] --> y
    #gps_readings[0] --> x

    #calculating target x and y as a multiple of 12
    targetx=x+dx[angle_to_idx[exact_angle(compass_value)]][0]
    targety=y+dy[angle_to_idx[exact_angle(compass_value)]][0]

    Scan_Camera()
    adjust_angle()
    while(robot.step(timestep) != -1):
        cnt += 1

        Scan_Camera()
        check_if_finished()
        
        get_gps_readings()
        ############################Shofy de ya sondos, did I put it in the right place of the code?#####################
        diff= abs(x-(gps_readings[0]-xstart)) + abs(y-(gps_readings[2]-ystart))
        # print("diff",diff)
        number_to_color={ -1:0, 6:2, 2:'r', 3:'g', 4:'b', 5:'p', 7:'o', 8:'y', 9:'3'}

        if (diff>=0.8):
            cond, rakam = color_sensor_detector_and_color(coords[targetx][targety])
        else:
            cond, rakam = color_sensor_detector_and_color(coords[x][y])
        adjust_angle()
        if labasna(targetx,targety,x,y) or rakam == 6:
            bad_node = Create_Node(targetx, targety)
            bad_id = coords[targetx][targety]
            visited[bad_node.id] = True
            bad_nodes.append(bad_id)
            global_bad = True

            return
        
        # if cnt > 5:
        #     cnt = 0
        #     x2, y2 = get_current_xy()

        #     if abs(x2 - x) < 3 and abs(y2 - y) < 3:
        #         cur_node = Node_of_ID[coords[x][y]]
        #         node2, list_of_angles = cur_node.get_nearest()

        #         move_with_angles(list_of_angles)
        #         break

        #     x = x2
        #     y = y2
        
        ######################################################################################################################
        # print("ELRAKAAMMMMMMMM",rakam, ' ', cond, " ", target_color, ' ', number_to_color[rakam])
        # if((cond and target_color!=number_to_color[rakam]) or rakam==6):  # raye7 area tanya and ana msh 3yzah yro7ha
        #     bad_node = Create_Node(targetx, targety)
        #     if(rakam == 6):
        #         visited[bad_node.id] = True
        #         bad_nodes.append(bad_node.id)
        #     else:
        #         list_of_colors.append(bad_node.id)

        #     stop()
        #     rotate_180()
        #     move_tile_elfatet(dx[angle_to_idx[exact_angle(compass_value)]][0])
        #     adjust_angle()
        #     target_color=0
        #     return 
        
        Scan_Camera()
        adjust_angle()
        move_forward()
        
        if dx[angle_to_idx[exact_angle(compass_value)]][0] == 0 and abs(targety-(gps_readings[2]-ystart)) < 0.25:
            stop()
            break

        if dy[angle_to_idx[exact_angle(compass_value)]][0] == 0 and abs(targetx-(gps_readings[0]-xstart)) < 0.25:
            stop()
            break

    # print("moved 1 odam")
    return
    

def move_tile_forward2():
    global coords, ID, Node_of_ID, visited, adj, target_color, tile_distance,dx,dy
    
    adjust_angle()

    
    get_gps_readings()
    get_compass_value()

    #calculating el starting tile as a multiple of 12
    x, y = get_current_xy()
    #gps_readings[2] --> y
    #gps_readings[0] --> x

    #calculating target x and y as a multiple of 12
    targetx=x+dx[angle_to_idx[exact_angle(compass_value)]][0]
    targety=y+dy[angle_to_idx[exact_angle(compass_value)]][0]

    Scan_Camera()
    adjust_angle()
    while(robot.step(timestep) != -1):
        
        get_gps_readings()
        ############################Shofy de ya sondos, did I put it in the right place of the code?#####################
        diff= abs(x-(gps_readings[0]-xstart)) + abs(y-(gps_readings[2]-ystart))
        adjust_angle()
        ######################################################################################################################
        # print("ELRAKAAMMMMMMMM",rakam, ' ', cond, " ", target_color, ' ', number_to_color[rakam])
        
        check_if_finished()

        Scan_Camera()
        adjust_angle()
        move_forward()
        
        if dx[angle_to_idx[exact_angle(compass_value)]][0] == 0 and abs(targety-(gps_readings[2]-ystart)) < 0.25:
            stop()
            break

        if dy[angle_to_idx[exact_angle(compass_value)]][0] == 0 and abs(targetx-(gps_readings[0]-xstart)) < 0.25:
            stop()
            break

    # print("moved 1 odam")
    return 

def addNodes():
    global coords, ID, Node_of_ID, visited, adj
    x, y = get_current_xy()
    id = coords[x][y]
    # print(id)
    cur_node = Node_of_ID[id]

    if(id == 0):
        cur_node = Create_Node(x, y)
        id = cur_node.id
        coords[x][y] = id
        Node_of_ID[cur_node.id] = cur_node
        visited[cur_node.id] = False
    visited[cur_node.id] = True

    # print("Actually node (", x, ", ", y, ") will go to:")

    for i in range(4):
        if (check_lidar(i)): 
            new_x = x + dx[angle_to_idx[exact_angle(compass_value)]][i]
            new_y = y + dy[angle_to_idx[exact_angle(compass_value)]][i]

            if(coords[new_x][new_y] != 0):
                id2 = coords[new_x][new_y]
                node2 = Node_of_ID[id2]
                if node2 in adj[cur_node.id]:
                    # print(node2.x, node2.y, " bad")
                    continue

                # print("Adding neighbor: ", id2, " to node: ", cur_node.id)
                # print(node2.x, node2.y, " good1")
                cur_node.add_neighbor(node2)
                continue
            
            # print(new_x, new_y, " good2")
            node2 = Create_Node(new_x, new_y)
            cur_node.add_neighbor(node2)

dl=[0,128,384,256]
#front, right, back, left

#checks that the lidar angle is from 0 to 511
def lidar_in_range(x):
    if (x<0):
        x+=512
    return x

#checks the interval from -10 to 10 of the front of the robot lidar to detect 7tkfyh y3dy wla la2
#MSHTAGHALETSH FA RG3TAHA TANY TTCHECK 3 RAYS, NEEDS TO BE EDITED
# def check_lidar(x):
#     f=True
#     for i in range(-50, 50):
#         f&=(lidar_value[2][lidar_in_range(dl[x]+i)]>6.9)
#         #old condition khalyha just in case 3shan lesa mgrbtsh el robot
#         #kant bttcheck 3 rays only
#     print("lidar OF",x,"is",f, "w awl value",lidar_value[2][dl[x]]," lidar +30",lidar_value[2][dl[x]+30]," lidar - 30",lidar_value[2][lidar_in_range(dl[x] - 30)])
#     # if (lidar_value[2][dl[x]]>7 and lidar_value[2][dl[x]+30]>7 and lidar_value[2][lidar_in_range(dl[x] - 30)]>6):
#         # return True

#     # return False
#     return f
############final youssef##############

# def check_lidar(x):
#     f=True
#     # print("LIDARRRR: ")
#     for i in range(-30, 31):
#         # print(lidar_value[2][dl[x]+i])
#         f&=(lidar_value[2][dl[x]+i] >= 7)
#     return f

######def check_lidar(x):
    # f=True
    ############# el akhira bta3t youssseffff#################
    # print("LIDARRRR: ")
    # for i in range(-80, 80):
        # print(lidar_value[2][dl[x]+i])
        # print("i is",i,"f etgah",dl[x],"lidar_value",lidar_value[2][lidar_in_range(dl[x]+i)])
        # f&=(lidar_value[2][lidar_in_range(dl[x]+i)] >= 5.8)

    # for i in range(-30, 31):
        # print(lidar_value[2][lidar_in_range(dl[x]+i)])
        # f&=(lidar_value[2][lidar_in_range(dl[x]+i)] >= tile_distance + 3.5)
      #  old condition khalyha just in case 3shan lesa mgrbtsh el robot
      #  kant bttcheck 3 rays only
    ####if (lidar_value[2][dl[x]]>7 and lidar_value[2][dl[x]+30]>6 and lidar_value[2][lidar_in_range(dl[x] - 30)]>6):
    #####    return True
    # print("lidar OF",x, "awl value",lidar_value[2][dl[x]]," lidar +30",lidar_value[2][dl[x]+30]," lidar - 30",lidar_value[2][lidar_in_range(dl[x] - 30)])
    # if (lidar_value[2][dl[x]]>7 and lidar_value[2][dl[x]+30]>7 and lidar_value[2][lidar_in_range(dl[x] - 30)]>7):
    #     return True

    # if f:
    #     print("omg it's free")
    # else:
    #     print("omg it's blocked")

    # return f
    #####return False

def get_time_remaining():
    global emitter, receiver
    message = struct.pack('c', 'G'.encode()) # message = 'G' for game information
    emitter.send(message) # send message

    cur = None

    if receiver.getQueueLength() > 0: # If receiver queue is not empty
        receivedData = receiver.getBytes()
        # print(receivedData, "received G")
        # if len(receivedData) != 12:
        #     print(f"Expected 12 bytes but got {len(receivedData)} bytes")
        #     # or handle the error appropriately
        # else:
        try:
            tup = struct.unpack('c f i', receivedData[:12]) # Parse data into char, float, int
            if tup[0].decode("utf-8") == 'G':
                cur = tup[2]
        
        except Exception as e:
            pass
            # print("Error unpacking data:", e)
            # receiver.clear() # Clear the receiver queue
            # Handle the error appropriately, e.g., log it or raise an exception

        receiver.nextPacket()

    # print(cur)
    return cur


def check_lidar(x):
    get_lidar_value()
    f=True
    #  print("LIDARRRR: ")
    for i in range(-30, 31):
        f&=(lidar_value[2][dl[x]+i] >= 7)
    
    
    # get_lidar_value()
    # if (x==1):
    #     print("new lidar readings:")
    #     for i in range(-30, 31):

    #         print("i is",i,"lidar_value",lidar_value[2][dl[x]+i])

    return f

#function bndyha instructions fa tro7 ll nearest non visited cell
def move_with_angles(list_of_angles):
    global global_bad
    if list_of_angles is None:
        return
    for angle in list_of_angles:
        if(global_bad):
            break
        check_if_finished()
        shortest_90(angle)
        check_if_finished()
        move_tile_forward()
    global_bad = False

def check_if_finished():
    time = get_time_remaining()

    if time is None:
        return 480

    if(time <= 6):
        Generate_all_the_hmm_map()
        robot.step(2000)
        emitter.send(bytes('E', "utf-8"))
        return

def follow_instructions_3emyany(list_of_angles):
    if list_of_angles is None:
        return
    for angle in list_of_angles:
        check_if_finished()
        shortest_90(angle)
        check_if_finished()
        move_tile_forward2()

priority_tiles=[[],['b','y','g'],['b','p','o'],['y','p','r'],['g','o','r']]
priority_areas=[[],[2,3,4],[1,3,4],[1,2,4],[1,2,3]]
khalasna = False
#funciton en el robot yfdal mask ymino 3shan y2ra as much walls as possiblr
def move_right_hand():
    global coords, ID, Node_of_ID, visited, adj,target_node, target_color, current_area_number, khalasna, list_of_colors
    if (robot.step(timestep)==-1):
        return
    
    get_lidar_value()
    get_compass_value()
    Scan_Camera()
    adjust_angle()

    check_if_finished()

    addNodes()

    for i in range(1, 25):
        node = Node_of_ID[i]
        if(node == None):
            continue
        # print(node.x, ", ", node.y, ": ")
        for j in range(len(adj[i])):
            pass
            # print("(", Node_of_ID[adj[i][j]].x, ", ", Node_of_ID[adj[i][j]].y, "), ")
            

    x, y = get_current_xy()

    # print("im in move right hand")

    good = [False] * 4
    for i in range(4):
        now_x = x + dx[angle_to_idx[exact_angle(compass_value)]][i]
        now_y = y + dy[angle_to_idx[exact_angle(compass_value)]][i]
        now_id = coords[now_x][now_y]

        # print(i, ": ", now_id)

        good[i] = (check_lidar(i) & (not visited[now_id]))

    if(good[1]):
        # print("OHHH YESSSS right is free")
        rotate_right()
    elif(good[0]):
        # print("OHHH YESSSS front is free")
        pass
    elif(good[2]):
        # print("OHHH YESSSS left is free")
        rotate_left()
    elif(good[3]):
        # print("OHHH YESSSS back is free")
        rotate_180()
    else:  # all of them are visited
        # print("now all are visited")
        cur_id = coords[x][y]
        cur_node = Node_of_ID[cur_id]
        target_node, list_of_angles = cur_node.get_nearest()
        area_changed=-1 
        # if target_node is None:  # WE NEED TO HANDLE THIS CASE
        #     # print("SHITTTTTT ANA KHALAST EL TILES AROOOO7 FENNNNN")
        #     area_done[current_area_number]=True

        #     for j in range(0,2):  #0,1,2 msh 7ncheck area 4 wla 7anro7laha
        #         Scan_Camera()
        #         adjust_angle()
        #         # print("el robot geh hena w dy el values", color_done[priority_tiles[current_area_number][j]]," ",priority_tiles[current_area_number][j]," ", area_done[priority_areas[current_area_number][j]]," ",priority_areas[current_area_number][j])
        #         if (color_done[priority_tiles[current_area_number][j]]==True and area_done[priority_areas[current_area_number][j]]==False):
        #             # print("roo7", priority_tiles[current_area_number][j],"a333",current_area_number,j,priority_tiles)
        #             target_node, list_of_angles = cur_node.get_color_tile(priority_tiles[current_area_number][j])
        #             # print("hena oho")
        #             print("ana raye7 el blue")
        #             target_color = priority_tiles[current_area_number][j]
        #             # current_area_number = priority_areas[current_area_number][j]
        #             area_changed=priority_areas[current_area_number][j]
        #             # print("AHHHHHHHHHHHHHH: ", target_node.x, target_node.y, " ", target_color)
        #             Scan_Camera()
        #             adjust_angle()
        #             break 

        if target_node is None: 
            #3ayz arg3 area 1
            target_node, list_of_angles = cur_node.rawa7ony()
            # target_color = mp[target_node.id][0][0]
            area_changed=1
            follow_instructions_3emyany(list_of_angles)
            khalasna = True
            Generate_all_the_hmm_map()
            robot.step(2000)
            emitter.send(bytes('E', "utf-8"))
            Scan_Camera()
            adjust_angle()
            return
                #####KHAlAS EXITTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT 
                ###########NEEEDS EDITING
            
            # if(current_area_number == 1):
            #     #shof el 3 priorities bl tartib bto3 el one b 7aga zy
            #     # tile priority[current_area_number][0] shofto abl keda wla la2a b color_done={} el ana ,definaha fo2
            #     #  w 23ml cur_node.get_color_tile() 23mlha zy .get_nearest bs 7ot fl conditon eno bydwr 3la tile el lonha kza msh awl wa7da not visited
            #     a1 = True
                # if(a2 == False and ):



            # print("ra7 el mlawena ely twadih area tanya")
        if list_of_angles is None:
            khalasna = True
        # print("now 7aymshy bel angles")
        move_with_angles(list_of_angles)
        if (area_changed!=-1):
            current_area_number=area_changed

        addNodes()
        
        if (khalasna==True):
            Generate_all_the_hmm_map()
            robot.step(2000)
            emitter.send(bytes('E', "utf-8"))
            return
        # print("now we will move right hand")
        move_right_hand()
        return

    move_tile_forward()
    move_right_hand()
    return

#endregion

##########################IMAGE DETECTION HANY MA7ADESH Y7AWL YFHAMO WLA Y3DL FYH GHER HANYYYYYYY########################

#region detecting hazards and victims
def Scan_Camera():
    image = right_camera.getImage()
    image2= left_camera.getImage()
    if image is not None:
        img = np.frombuffer(image, np.uint8).reshape((right_camera.getHeight(), right_camera.getWidth(), 4))
        bgr = img[:, :, :3]
        
        # Check for hazards/victims
        checkingfirst = check_first(right_camera)
        if checkingfirst == "LETTER":
            detect_letters(right_camera, 'r')
        elif checkingfirst == "HAZARD":
            hazard_image_detection(right_camera, 'r')

    if image2 is not None:
        img2 = np.frombuffer(image2, np.uint8).reshape((left_camera.getHeight(), left_camera.getWidth(), 4))
        bgr2 = img2[:, :, :3]

        checkingfirst= check_first(left_camera)
        if checkingfirst == "LETTER":
            detect_letters(left_camera, 'l')
        elif checkingfirst == "HAZARD":
            hazard_image_detection(left_camera, 'l')
def hazard_image_detection(camera, camera_letter):
    global last_hazard_time
    
    image = camera.getImage()
    
    
    img = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    bgr = img[:, :, :3]
    visualization_img = bgr.copy()
    
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    
    kernel = np.ones((3, 3), np.uint8)
    red_mask = cv2.inRange(hsv, (0, 120, 100), (10, 255, 255)) | cv2.inRange(hsv, (160, 120, 100), (180, 255, 255))
    yellow_mask = cv2.inRange(hsv, (20, 120, 100), (30, 255, 255))
    white_mask = cv2.inRange(gray, 200, 255)
    black_mask = cv2.inRange(gray, 0, 50)
    
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
    
    roi_area = bgr.shape[0] * bgr.shape[1]
    
    hazard = None
    hazard_contour = None
    percentage_info = ""
    
    # FIRST: Look for diamond/rhombus shapes for F and O hazards
    edges = cv2.Canny(gray, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        if cv2.contourArea(cnt) < 0.05 * roi_area:
            continue
            
        epsilon = 0.04 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        
        # Get the bounding rectangle and check if it's in the upper portion of the image
        # x, y, w, h = cv2.boundingRect(cnt)
        # if y > (gray.shape[0] * 0.25):  # Skip if the shape is in the bottom 40% of the image
        #     continue            
        if len(approx) == 4:  # 4-sided shape
            rect = cv2.minAreaRect(cnt)
            angle = rect[2]
            
            if 35<= abs(angle) <= 60:  # Diamond/rhombus angle check
                rhombus_mask = np.zeros(gray.shape, np.uint8)
                cv2.drawContours(rhombus_mask, [approx], -1, 255, -1)
                
                # Calculate color areas INSIDE the rhombus only
                rhombus_red = cv2.countNonZero(cv2.bitwise_and(red_mask, rhombus_mask))
                rhombus_yellow = cv2.countNonZero(cv2.bitwise_and(yellow_mask, rhombus_mask))
                rhombus_total = cv2.countNonZero(rhombus_mask)
                
                if rhombus_total > 0:
                    red_ratio = rhombus_red / rhombus_total
                    yellow_ratio = rhombus_yellow / rhombus_total
                    # print(f"Debug - Red ratio: {red_ratio:.2%}, Yellow ratio: {yellow_ratio:.2%}")
                    
                    # Organic Peroxide (O) - both red and yellow inside with significant ratios
                    if red_ratio > 0.3 and yellow_ratio > 0.3:
                        hazard = 'O'
                        percentage_info = f"R:{red_ratio:.1%} Y:{yellow_ratio:.1%}"
                        hazard_contour = approx
                        break
                    
                    # Flammable (F) - significant red ratio
                    elif red_ratio > 0.3:
                        hazard = 'F'
                        percentage_info = f"R:{red_ratio:.1%}"
                        hazard_contour = approx
                        break    # SECOND: If no F/O found in rhombus shapes, check for P and C hazards (original method)
     
    if not hazard:
        edges = cv2.Canny(gray, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) < 0.05 * roi_area:
                continue
                
            epsilon = 0.04 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            if len(approx) == 4:
                rect = cv2.minAreaRect(cnt)
                angle = rect[2]
                
                if 30 <= abs(angle) <= 60:
                    rhombus_mask = np.zeros(gray.shape, np.uint8)
                    cv2.drawContours(rhombus_mask, [approx], -1, 255, -1)
                    
                    rhombus_white = cv2.countNonZero(cv2.bitwise_and(white_mask, rhombus_mask))
                    rhombus_black = cv2.countNonZero(cv2.bitwise_and(black_mask, rhombus_mask))
                    rhombus_total = rhombus_white + rhombus_black
                    
                    if rhombus_total > 0:
                        white_ratio = rhombus_white / rhombus_total
                        black_ratio = rhombus_black / rhombus_total
                        print(f"Debug - White ratio: {white_ratio:.2%}, Black ratio: {black_ratio:.2%}")
                        
                        if white_ratio > 0.7 and white_ratio > black_ratio:
                            hazard = 'P'
                            percentage_info = f"W:{white_ratio:.1%} B:{black_ratio:.1%}"
                        elif 0.2<=black_ratio<=0.999999 and black_ratio> white_ratio:
                            hazard = 'C'
                            percentage_info = f"W:{white_ratio:.1%} B:{black_ratio:.1%}"
                        
                        if hazard:
                            hazard_contour = approx
                            break    
    # Visualization and reporting
    if hazard and hazard_contour is not None:
        if hazard in ['P', 'C', 'F', 'O']:
            pass
            # print(f"Detected {hazard}: {percentage_info}")
            cv2.drawContours(visualization_img, [hazard_contour], -1, (0, 255, 0), 2)
        
        cv2.putText(visualization_img, f"{hazard} {percentage_info}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # cv2.imshow("Hazard Detection", visualization_img)
    
    if hazard:
        stop()
        robot.step(2000)
        victim_type = bytes(hazard, 'utf-8')
        position = gps.getValues()
        x_pos = int(position[0] * 100)
        y_pos = int(position[2] * 100)
        message = struct.pack("i i c", x_pos, y_pos, victim_type)
        emitter.send(message)
        shahdx,shahdy=get_current_xy()
        get_compass_value()
        do_shahds_function_for_mapping(Node_of_ID[coords[shahdx][shahdy]], hazard, compass_value,camera_letter)
        # print(f"Detected {hazard} at ({x_pos}, {y_pos}) - {percentage_info}")
        robot.step(1000)
        left_wheel.setVelocity(1)
        right_wheel.setVelocity(1)
    
    # cv2.waitKey(1)
    return hazard

def detect_letters(camera, camera_letter):
    global detected_letter, is_stopped, stop_start_time

    image = camera.getImage()
    if image is None:
        return False

    image = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    image_bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)
    
    kernel = np.ones((3, 3), np.uint8)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
    
    white_contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    letter_detected = False
    letter = None

    for contour in white_contours:
        area = cv2.contourArea(contour)
        if area < 100:
            continue
            
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w)/h
            
            if 0.8 <= aspect_ratio <= 1.2:
                white_region_mask = np.zeros_like(white_mask)
                cv2.drawContours(white_region_mask, [contour], -1, 255, -1)
                white_region = cv2.bitwise_and(image_bgr, image_bgr, mask=white_region_mask)
                
                gray_region = cv2.cvtColor(white_region, cv2.COLOR_BGR2GRAY)
                gray_region = cv2.GaussianBlur(gray_region, (3, 3), 0)
                _, black_mask = cv2.threshold(gray_region, 50, 255, cv2.THRESH_BINARY_INV)
                
                black_contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for black_contour in black_contours:
                    x, y, w, h = cv2.boundingRect(black_contour)
                    
                    if (w < 10 or h < 10 or 
                        w > 0.9*white_region_mask.shape[1] or 
                        h > 0.9*white_region_mask.shape[0]):
                        continue
                        
                    if (x > 0 and y > 0 and 
                        (x + w) < camera.getWidth() and 
                        (y + h) < camera.getHeight()):
                        
                        letter_region = black_mask[y:y + h, x:x + w]
                        
                        part_height = h // 3
                        top_part = letter_region[0:part_height, :]
                        middle_part = letter_region[part_height:2 * part_height, :]
                        bottom_part = letter_region[2 * part_height:, :]
                        
                        top_contours, _ = cv2.findContours(
                            top_part, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        middle_contours, _ = cv2.findContours(
                            middle_part, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        bottom_contours, _ = cv2.findContours(
                            bottom_part, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        
                        letter = None
                        top_count = len(top_contours)
                        middle_count = len(middle_contours)
                        bottom_count = len(bottom_contours)
                        
                        if (top_count == 2 and middle_count == 1 and bottom_count == 2 and
                            cv2.contourArea(top_contours[0]) > 5 and
                            cv2.contourArea(top_contours[1]) > 5):
                            letter = "H"
                            
                        elif (top_count == 1 and middle_count == 1 and bottom_count == 1 and
                              cv2.contourArea(top_contours[0]) > 15):
                            letter = "S"
                            
                        elif (top_count == 2 and middle_count == 2 and bottom_count == 1 and
                              cv2.contourArea(bottom_contours[0]) > 10):
                            letter = "U"
                        else:
                            sign = ["H", "S", "U"]
                            letter = random.choice(sign)

                        if letter is not None:
                            cv2.rectangle(image_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            cv2.putText(image_bgr, letter, (x, y - 10), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                            if letter in ["H", "S", "U"]:
                                letter_detected = True
                                detected_letter = letter
                                # print(f"Detected letter: {letter}")
                                
                                stop()
                                if detected_letter:
                                    robot.step(2000)
                                    # print("I need to stop here")
                                    victimetype = bytes(detected_letter, 'utf-8')
                                    position = gps.getValues()
                                    x_pos = int(position[0] * 100)
                                    y_pos = int(position[2] * 100)
                                    message = struct.pack("i i c", x_pos, y_pos, victimetype)
                                    emitter.send(message)
                                    shahdx,shahdy=get_current_xy()
                                    get_compass_value()
                                    do_shahds_function_for_mapping(Node_of_ID[coords[shahdx][shahdy]], detected_letter, compass_value,camera_letter)
                                    # print("I need to stop here also so i can send message")
                                    # print("astana b2a e4waya")
                                    robot.step(1000)                            
                                    # print(f"Sent letter: {detected_letter} (Position: {x_pos}, {y_pos})")

    # cv2.imshow("Letter Detection", image_bgr)
    cv2.waitKey(1)
    return letter_detected

def check_first(camera):
    """Determines whether to process as letter or hazard"""
    image = camera.getImage()
    img = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    bgr = img[:, :, :3]
    
    # White detection
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 30, 255]))
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    
    # Find white contours
    white_contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in white_contours:
        if cv2.contourArea(contour) < 100:
            continue
            
        # Check if square-like
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04*perimeter, True)
        if len(approx) == 4:  # Quadrilateral
            x,y,w,h = cv2.boundingRect(contour)
            if 0.7 <= w/h <= 1.3:  # Good aspect ratio
                # print("LETTER M4 mot2ked ")
                return 'LETTER'  # Trigger letter detection
    
    # If no valid white square found
    # print("HAZARD M4 mot2ked ")
    return 'HAZARD'  # Trigger hazard detection


#endregion
##################################################################### mappping ###################################################################
##########################MAPPING SHAHD SEBOH L SHAHDDD#################################
#region mapping
###########################################################################
# id=coords[x][y] #object id
# node = Node_od_ID[id] #object node
# node.x
# node.y
# node.id

# node.add_neighbor
# node.get_nearest

# adj[id] --> returns array of ids of nodes

# color_map = [0] * 251


# adj[id]
# adj[id]
# Hani should call the function do_shahds_function_for_mapping(cur_node,value,cur_comp_val,camera_direction)
# after every sign detection please, thank you in advance!
#should also add self.erea  = current_area_number in class node

min_x_coord = 100000000
min_y_coord = 100000000
max_x_coord = -100000000
max_y_coord = -100000000
max_row_length = 0
max_column_length = 0
final_adjacency_list = [[0 for _ in range(1000)] for _ in range(1000)]

def is_up(node1, node2):
    if node1.x==node2.x and node1.y > node2.y:
        return True
    if node1.x==12 and node1.y==0 and node2.x==0 and node2.y==0:
        print("ana f3ln dakhalt hna")
    return False
def is_down(node1, node2):
    if node1.x==node2.x and node1.y < node2.y:
        return True
    return False
def is_left(node1, node2):
    if node1.y==node2.y and node1.x > node2.x:
        return True
    return False
def is_right(node1, node2):
    if node1.y==node2.y and node1.x < node2.x:
        return True
    return False
def do_for_full_tile(up,down,left,right,starting_x, starting_y,node_id):
    global final_adjacency_list,mp
    if up==False:
        final_adjacency_list[starting_x][starting_y]='1'
        final_adjacency_list[starting_x][starting_y+1]='1'
        final_adjacency_list[starting_x][starting_y+2]='1'
        final_adjacency_list[starting_x][starting_y+3]='1'
        final_adjacency_list[starting_x][starting_y+4]='1'
    if down==False:
        final_adjacency_list[starting_x+4][starting_y]='1'
        final_adjacency_list[starting_x+4][starting_y+1]='1'
        final_adjacency_list[starting_x+4][starting_y+2]='1'
        final_adjacency_list[starting_x+4][starting_y+3]='1'
        final_adjacency_list[starting_x+4][starting_y+4]='1'
    if left==False:
        final_adjacency_list[starting_x][starting_y]='1'
        final_adjacency_list[starting_x+1][starting_y]='1'
        final_adjacency_list[starting_x+2][starting_y]='1'
        final_adjacency_list[starting_x+3][starting_y]='1'
        final_adjacency_list[starting_x+4][starting_y]='1'
    if right==False:
        final_adjacency_list[starting_x][starting_y+4]='1'
        final_adjacency_list[starting_x+1][starting_y+4]='1'
        final_adjacency_list[starting_x+2][starting_y+4]='1'
        final_adjacency_list[starting_x+3][starting_y+4]='1'
        final_adjacency_list[starting_x+4][starting_y+4]='1'
    if (node_id not in mp):
            mp[node_id] = [['0', '0', '0'], ['0', '0', '0'], ['0', '0', '0']]  # Initialize the tile shape

    final_adjacency_list[starting_x+1][starting_y+1]=mp[node_id][0][0]
    final_adjacency_list[starting_x+1][starting_y+3]=mp[node_id][0][0]
    final_adjacency_list[starting_x+3][starting_y+1]=mp[node_id][0][0]
    final_adjacency_list[starting_x+3][starting_y+3]=mp[node_id][0][0]
def do_for_half_tile(up,down,left,right,starting_x, starting_y,node_id):
    global final_adjacency_list,mp
    if up==False:
        final_adjacency_list[starting_x][starting_y]='1'
        final_adjacency_list[starting_x][starting_y+1]='1'
        final_adjacency_list[starting_x][starting_y+2]='1'
    if down==False:
        final_adjacency_list[starting_x+2][starting_y]='1'
        final_adjacency_list[starting_x+2][starting_y+1]='1'
        final_adjacency_list[starting_x+2][starting_y+2]='1'
    if left==False:
        final_adjacency_list[starting_x][starting_y]='1'
        final_adjacency_list[starting_x+1][starting_y]='1'
        final_adjacency_list[starting_x+2][starting_y]='1'
    if right==False:
        final_adjacency_list[starting_x][starting_y+2]='1'
        final_adjacency_list[starting_x+1][starting_y+2]='1'
        final_adjacency_list[starting_x+2][starting_y+2]='1'
    if (node_id not in mp):
            mp[node_id] = [['0', '0', '0'], ['0', '0', '0'], ['0', '0', '0']]  # Initialize the tile shape
    final_adjacency_list[starting_x+1][starting_y+1]=mp[node_id][0][0]
def fill_with_stars(starting_x, starting_y):
    global final_adjacency_list
    for i in range(starting_x, starting_x+4):
        for j in range(starting_y, starting_y+4):
            final_adjacency_list[i][j]='*'
def valid_points(i,j):
    if i>=min_x_coord and i<=max_x_coord and j>=min_y_coord and j<=max_y_coord:
        return True
    return False

def generate_map2():
    global coords, ID, Node_of_ID, visited, adj, final_adjacency_list
    global min_x_coord, min_y_coord, max_x_coord, max_y_coord
    global mp, max_row_length, max_column_length
    visited_for_shahd = {}
    for j in range(min_y_coord, max_y_coord+12, 12):  # Loop through columns (0 to 84 with step 12)
        for i in range(min_x_coord, max_x_coord+12, 12):
            if visited[coords[i][j]]==False:
                # # print("The current point is", i,j)  # Loop through rows (0 to 84 with step 12)
                # if i%12==0 and j%12==0 and (valid_points(i-12,j)==False or visited[coords[i-12][j]]==False or valid_points(i+12,j)==False or visited[coords[i+12][j]]==False):
                #     starting_y=int((current_node.x-min_x_coord)/12)*4 + 1
                #     starting_x=int((current_node.y-min_y_coord)/12)*4 + 1
                #     fill_with_stars(starting_x, starting_y)
                continue
            print("The current point is", i,j)  # Loop through rows (0 to 84 with step 12)
            node_id = coords[i][j]  # Get the node ID from the coordinates
            up=False
            down=False
            left=False
            right=False
            for adj_node_id in adj[node_id]:
                visited_for_shahd[(node_id,adj_node_id)] = True
                adj_node=Node_of_ID[adj_node_id]
                if is_up(Node_of_ID[node_id], adj_node):
                    up=True
                if is_down(Node_of_ID[node_id], adj_node):
                    down=True
                if is_left(Node_of_ID[node_id], adj_node):
                    left=True
                if is_right(Node_of_ID[node_id], adj_node):
                    right=True
            current_node = Node_of_ID[node_id]
            if Node_of_ID[node_id].area==1:
                starting_y=int((current_node.x-min_x_coord)/12)*4
                starting_x=int((current_node.y-min_y_coord)/12)*4
                do_for_full_tile(up,down,left,right,starting_x, starting_y,node_id)
            else:
                starting_y=int((current_node.x-min_x_coord)/6)*2
                starting_x=int((current_node.y-min_y_coord)/6)*2
                do_for_half_tile(up,down,left,right,starting_x, starting_y,node_id)
            
def Generate_all_the_hmm_map():
    global coords, ID, Node_of_ID, visited, adj, final_adjacency_list
    global mp, hazards_positions
    generate_map2()
    strating_node=Node_of_ID[1]
    strting_node_y_in_adj_list=int((strating_node.x-min_x_coord)/12)*4
    strting_node_x_in_adj_list=int((strating_node.y-min_y_coord)/12)*4
    final_adjacency_list[strting_node_x_in_adj_list+1][strting_node_y_in_adj_list+1]='5'
    final_adjacency_list[strting_node_x_in_adj_list+1][strting_node_y_in_adj_list+3]='5'
    final_adjacency_list[strting_node_x_in_adj_list+3][strting_node_y_in_adj_list+1]='5'
    final_adjacency_list[strting_node_x_in_adj_list+3][strting_node_y_in_adj_list+3]='5'

    ####HAZARDSSSSSSSSS
    for sign_info in hazards_positions:
        # print("The sign info is", sign_info)
        node, value, direction = sign_info
        x = int(node.y / 12) * 4
        y = int(node.x / 12) * 4
        if direction == 'u':
            final_adjacency_list[x][y+2] = value
        elif direction == 'd':
            final_adjacency_list[x + 4][y+2] = value
        elif direction == 'l':
            final_adjacency_list[x+2][y] = value
        elif direction == 'r':
            final_adjacency_list[x+2][y + 4] = value
    
    submatrix = np.array(final_adjacency_list)  # Convert to NumPy array
    s = submatrix.shape
    ## Get shape as bytes
    s_bytes = struct.pack('2i', *s)

    ## Flattening the matrix and join with ','
    flatMap = ','.join(map(str, submatrix.flatten()))  # Flatten and join as a string
    ## Encode
    sub_bytes = flatMap.encode('utf-8')

    ## Add together, shape + map
    a_bytes = s_bytes + sub_bytes

    ## Send map data
    robot.step(2000)
    emitter.send(a_bytes)

    # STEP3 Send map evaluate request
    map_evaluate_request = struct.pack('c', b'M')
    emitter.send(map_evaluate_request)

    # STEP4 Send an Exit message to get Map Bonus
    ## Exit message
    exit_mes = struct.pack('c', b'E')
    emitter.send(exit_mes)
    robot.step(2000)


#region lack of progress bta3tna
hazards_positions = []
def assign_nearest_angle(angle):
    # List of target angles
    target_angles = [0, 90, -90, 180, -180]
    
    # Find the target angle with the smallest difference
    nearest_angle = min(target_angles, key=lambda x: abs(angle - x))
    
    return nearest_angle
def given_sign_details_then_find_sign_direction(cur_comp_val,camera_direction):
    if (camera_direction=="r"):
        return cur_comp_val-90
    return cur_comp_val+90
def do_shahds_function_for_mapping(cur_node,value,cur_comp_val,camera_direction):
    angle = given_sign_details_then_find_sign_direction(cur_comp_val , camera_direction)
    angle = check_angle(assign_nearest_angle(angle))
    if angle==-180:
        angle=180
    if angle==90:
        hazards_positions.append((cur_node, value, 'l'))
    elif angle==-90:
        hazards_positions.append((cur_node, value, 'r'))
    elif angle==0:
        hazards_positions.append((cur_node, value, 'u'))
    elif angle==180:
        hazards_positions.append((cur_node, value, 'd'))
    # else:
    #     print("Feeeh Error kbeer call IT shahooda")

#region lack of progress bta3tna

def labasna(targetx,targety,x,y):
    # return False
    get_lidar_value()
    f=True
    for i in range(0, 512):
        # print(i," ",lidar_value[2][i])
        # print("lidar",i,lidar_value[2][i])
        f&= (lidar_value[2][i]>4.1)

    if f==False:
        bad_node = Create_Node(targetx, targety)
        visited[bad_node.id] = True

        stop()
        # print("eh dah e7na labasna")
        move_tile_elfatet_bdahro(dx[angle_to_idx[exact_angle(compass_value)]][0],x,y)
        # rotate_180()
        adjust_angle()
        target_color=0
        return True
    return False
#endregion
########################################################################################################################################################################
########################################################################################################################################################################
########################################################################################################################################################################
########################################################################################################################################################################

#region main code
##################################################################### main code ########################################################################


################
robot.step(timeStep)
    
get_gps_readings()
xstart=gps_readings[0]
ystart=gps_readings[2]
get_compass_value()
compass_start=compass_value

modifiedx, modifiedy = get_current_xy()
for i in range(-50, 51):
    for j in range(-50, 51):
        coords.setdefault(i * 6, {})[j * 6] = 0
mainNode = Node(modifiedx, modifiedy)

for i in range(1,5):
    area_done[i]=False

for i in range(0,5):
    color_done[i]=False

for i in "rgbpoy":
    color_done[i]=False



while 1:
    move_right_hand()


#############


#endregion
