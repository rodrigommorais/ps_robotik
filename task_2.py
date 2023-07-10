import numpy as np
import matplotlib.pyplot as plt

class Object:

    def __init__(self, form, x, y, color) -> None:
        self.form: str = form
        self.x: float = x
        self.y: float = y
        self.color: str = color


def getAngle(x1,y1,x2,y2):
    eps = 10**(-10)
    num = y2-y1
    den = x2-x1
    atan = np.arctan((y2-y1)/((x2-x1)+eps))
    if atan<0:
        if num>0 or den<0:
            atan = atan + np.pi
    else:
        if num<0 or den<0:
            atan+= np.pi
    return atan % (2*np.pi)


def select_next(start, objects_left):
    next_object: Object
    min_distance = 2

    for object_left in objects_left:
        object_left_distance = np.sqrt((object_left.x-float(start[0]))**2 + (object_left.y-float(start[1]))**2)
        if object_left_distance < min_distance:
            next_object = object_left
            min_distance = object_left_distance

    return next_object


def trajectory_for_object(start, next_object):

    x = next_object.x
    y = next_object.y
    d = 0.07
    square_points = [[x-d,y-d], [x-d,y+d], [x+d,y+d], [x+d,y-d]]
    min_distance = 2

    for i, point in enumerate(square_points):
        distance_to_point = np.sqrt((start[0]-point[0])**2 + (start[1]-point[1])**2)
        if distance_to_point < min_distance:
            goal = point
            min_distance = distance_to_point
            point_index = i

    num_discretization_step = round(min_distance*10)
    x_in_trajectory = np.linspace(start[0], goal[0], num_discretization_step)
    y_in_trajectory = np.linspace(start[1], goal[1], num_discretization_step)
    planned_trajectory = []

    for i in range (0,num_discretization_step):
        planned_trajectory.append([x_in_trajectory[i], y_in_trajectory[i]])

    for i, point in enumerate(square_points):
        if next_object.color == "red":
            next_point_index = (point_index+i+1)%4
        else:
            next_point_index = (point_index-i-1)%4
        planned_trajectory.append([square_points[next_point_index][0], square_points[next_point_index][1]])

    return planned_trajectory


def plan_trajectory(objects):

    objects_left = objects
    start = [0.05,0.05] 
    last_point =  start
    planned_trajectory = [start]
    count = 0

    while objects_left != []:

        next_object = select_next(last_point, objects_left)
        next_trajectory = trajectory_for_object(last_point, next_object)
        for point in next_trajectory:
            planned_trajectory.append(point)    
        last_point = planned_trajectory[-1]

        for i, o in enumerate(objects_left):
            if o == next_object:
                del objects_left[i]
                break

        count = count + 1        
        if count > 10000: 
            print("Error in planning the trajectory")
            break
    
    distance_to_start = np.sqrt((start[0]-last_point[0])**2 + (start[1]-last_point[1])**2)
    num_discretization_step = round(distance_to_start*10)

    x_in_trajectory = np.linspace(last_point[0], start[0], num_discretization_step)
    y_in_trajectory = np.linspace(last_point[1], start[1], num_discretization_step)

    for i in range (0,num_discretization_step):
        planned_trajectory.append([x_in_trajectory[i], y_in_trajectory[i]])

    print(planned_trajectory)

    return planned_trajectory


def plot_trajectory(planned_trajectory):

    x_plot = []
    y_plot = []

    plt.figure()
    for trajectory_point in planned_trajectory:
        x_plot.append(trajectory_point[0])
        y_plot.append(trajectory_point[1])
    
    plt.xlim([0, 1.485])
    plt.ylim([0, 1.485])
    plt.grid()
    plt.plot(x_plot, y_plot, '-o', color='orange')
    plt.title("Arena - Trajectory task 2")
    plt.show()

if __name__ == '__main__':
    objects = [Object("cube", 0.7, 0.7, "blue"), Object("cube", 1.2, 0.2, "red")]
    planned_trajectory = plan_trajectory(objects)
    plot_trajectory(planned_trajectory)