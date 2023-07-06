from scripts.motors_waveshare import set_speed
from scripts.pose import get_pose
import math
import time
import numpy as np

def straight(distance, speed, direction):

    speed_diference = 0 #Ajust after testing
    speed_l = -1*speed
    speed_r = -1*(speed+speed_diference)
    if direction < 0:
        speed_l = -1*speed_l
        speed_r = -1*speed_r

    c = 10 #Ajust after testing
    set_speed(speed_l, speed_r)
    time.sleep(c*distance/speed)
    set_speed(0, 0)

def rotate(angles, speed, direction):
    speed_l = -1*speed
    speed_r = -1*(speed)
    if direction == "clockwise":
        speed_r*=-1
    else:
        speed_l*=-1
        
    c = 10 #Ajust after testing
    set_speed(speed_l, speed_r)
    time.sleep(c*angles/speed)
    set_speed(0, 0)

def getAngle(x1,y1,x2,y2):
    eps = 10**(-10)
    num = y2-y1
    den = x2-x1
    atan = math.atan((y2-y1)/((x2-x1)+eps))
    if atan<0:
        if num>0 or den<0:
            atan = atan + math.pi
    else:
        if num<0 or den<0:
            atan+= math.pi
    return atan % (2*math.pi)

def getDistance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

def moveRobot(x1,y1,theta, x2, y2):

    angleForTravel = getAngle(x1,x2,y1,y2)
    print(angleForTravel, "angle for travel")
    angleToBeRotated = (angleForTravel - theta)%(2*math.pi)
    if angleToBeRotated >=1.5*math.pi or angleToBeRotated <= 0.5*math.pi:
        if angleToBeRotated >= 1.5*math.pi:
            angleToBeRotated -= 2*math.pi
            orientation = "clockwise"
        else:
            orientation = "anticlockwise"
        direction = 1
    else:
        if angleToBeRotated >= math.pi:
            angleToBeRotated -= math.pi
            orientation = "anticlockwise"
            direction = -1
        else:
            angleToBeRotated -= math.pi
            orientation = "clockwise"
            direction = -1
    # if abs(angleToBeRotated) > 0.5:
    rotate(abs(angleToBeRotated),0.7,orientation)
    print("ROTATED by ", angleToBeRotated, orientation)
        # if abs(angleToBeRotated) > 0.8:
        #     return

    distance = getDistance(x1,y1,x2,y2)
    t = time.start()
    straight(distance*100,0.7,direction)
    t = time.end() -t


def drive_trajectory(planned_trajectory):

    current_pose = get_pose()
    x = current_pose.x
    y = current_pose.y

    for waypoint in planned_trajectory:

        dest_x = waypoint.x
        dest_y = waypoint.y

        travel_angle = getAngle(x, y, dest_x, dest_y)
        moveRobot(x, y, travel_angle, x, y)
        
        is_there_a_close_obstacle, obstacle_x, obstacle_y = check_obstacles()

        if is_there_a_close_obstacle == True:
            dodge_obstacle(x, y, travel_angle, obstacle_x, obstacle_y)
            current_pose = get_pose()
            x = current_pose.x
            y = current_pose.y
            travel_angle = getAngle(x, y, dest_x, dest_y)

        moveRobot(x, y, travel_angle, dest_x, dest_y)
            
        current_pose = get_pose()
        x = current_pose.x
        y = current_pose.y


def dodge_obstacle(x, y, travel_angle, obstacle_x, obstacle_y):

    angle_to_object = getAngle(x, y, obstacle_x, obstacle_y)
    distance_to_object = getDistance(x, y, obstacle_x, obstacle_y)

    if angle_to_object <= travel_angle:
        intermediate_angle = angle_to_object + math.pi/6
    else:
        intermediate_angle = angle_to_object - math.pi/6

    intermediate_x = x + np.cos(intermediate_angle)*distance_to_object
    intermediate_y = y + np.sin(intermediate_angle)*distance_to_object

    moveRobot(x, y, intermediate_angle, intermediate_x, intermediate_y)