import numpy as np
#from scripts.pose import get_pose, Pose, Object
#from scripts.motion import drive_trajectory
import matplotlib.pyplot as plt


class Pose:

    def __init__(self, x, y, angle) -> None:
        self.x: float = x
        self.y: float = y
        self.angle: float = angle

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


def select_next(pose, objects_left):
    next_object: Object
    min_distance = 3

    for object_left in objects_left:
        # print(object_left)
        object_left_distance = np.sqrt((object_left.x-pose.x)**2 + (object_left.y-pose.y)**2)
        if object_left_distance < min_distance:
            next_object = object_left
            min_distance = object_left_distance

    return next_object


def trajectory_until_object(pose, next_object):

    start = pose
    goal = Pose(next_object.x-0.08, next_object.y, 0)
    distance = np.sqrt((goal.x-start.x)**2 + (goal.y-start.y)**2)
    num_discretization_step = round(distance*10)

    x_in_trajectory = np.linspace(start.x, goal.x, num_discretization_step)
    y_in_trajectory = np.linspace(start.y, goal.y, num_discretization_step)
    angle_in_trajectory = np.full(num_discretization_step, np.tanh((goal.y-start.y)/(goal.x-start.x)))
    # print(angle_in_trajectory)
    # print(num_discretization_step)
    planned_trajectory = []

    for i in range (0,num_discretization_step):
        # print(i)
        # print(x_in_trajectory[i])
        # print(angle_in_trajectory[i])
        planned_trajectory.append(Pose(x_in_trajectory[i], y_in_trajectory[i], angle_in_trajectory[i]))

    # print(planned_trajectory)

    return planned_trajectory


def trajectory_around_object(pose, next_object):

    next_x_in_trajectory = pose.x
    next_y_in_trajectory = pose.y
    radius = 0.08
    center_x = next_x_in_trajectory + radius
    center_y = next_y_in_trajectory
    num_discretization_step = 6
    discretization_angle = (2*np.pi/num_discretization_step)
    print()
    planned_trajectory = []

    for i in range(1, num_discretization_step):
        previous_x_in_trajectory = next_x_in_trajectory
        previous_y_in_trajectory = next_y_in_trajectory
        next_x_in_trajectory = center_x - radius*np.cos(i*discretization_angle)
        next_y_in_trajectory = center_y + radius*np.sin(i*discretization_angle)
        print(next_x_in_trajectory, next_y_in_trajectory)
        angle_in_trajectory = getAngle(previous_x_in_trajectory, previous_y_in_trajectory, next_x_in_trajectory, next_y_in_trajectory)
        planned_trajectory.append(Pose(next_x_in_trajectory, next_y_in_trajectory, angle_in_trajectory))
    
    aux_list = []
    if next_object.color == "red":
        for trajectory_point in planned_trajectory:
            aux_list = [trajectory_point] + aux_list
        planned_trajectory = aux_list

    print(planned_trajectory)

    return planned_trajectory


def plot_trajectory(planned_trajectory):

    x_plot = []
    y_plot = []

    plt.figure()
    for trajectory_point in planned_trajectory:
        x_plot.append(trajectory_point.x)
        y_plot.append(trajectory_point.y)
    
    plt.xlim([0, 1.485])
    plt.ylim([0, 1.485])
    plt.grid()
    plt.plot(x_plot, y_plot, '-o', color='orange')
    plt.show()


def plan_trajectory(objects):

    objects_left = objects
    pose = Pose(0.05,0.05,0)  
    planned_trajectory = [pose]
    count = 0

    while objects_left != []:

        next_object = select_next(pose, objects_left)
        
        
        next_trajectory = trajectory_until_object(pose, next_object)
        planned_trajectory.extend(next_trajectory)
        pose = next_trajectory[-1]

        next_trajectory = trajectory_around_object(pose, next_object)
        planned_trajectory.extend(next_trajectory)  
        pose = next_trajectory[-1]

        for i, o in enumerate(objects_left):
            if o == next_object:
                del objects_left[i]
                break

        count = count + 1        
        if count > 10000: 
            print("Error in planning the trajectory")
            break
    
    next_trajectory = trajectory_until_object(pose, Pose(0.05+0.08,0.05,0))
    planned_trajectory.extend(next_trajectory)

    print(planned_trajectory)

    return planned_trajectory


if __name__ == '__main__':
    #current_pose = get_pose()
    objects = [Object("cube", 0.7, 0.7, "blue"), Object("cube", 1, 0.2, "red")]
    planned_trajectory = plan_trajectory(objects)
    plot_trajectory(planned_trajectory)