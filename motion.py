from scripts.motors_waveshare import set_speed
import math
import time

def straight(distance, speed, direction):

    speed_diference = 0.015 #Ajust after testing
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

def getAngleForTravel(x1,x2,y1,y2):
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
    global current_pose, t
    angleForTravel = getAngleForTravel(x1,x2,y1,y2)
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

    current_pose = [x2, y2, angleForTravel]
