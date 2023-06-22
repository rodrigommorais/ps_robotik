import numpy as np

from motion import moveRobot
from scripts.pose import get_pose


class Object:

    def __init__(self, form, x, y, color) -> None:
        self.form: str = form
        self.x: float = x
        self.y: float = y
        self.color: str = color


class Pose:

    def __init__(self, x, y, angle) -> None:
        self.x: float = x
        self.y: float = y
        self.angle: float = angle


def select_next(pose, objects_left):

    next_object: Object
    min_distance = 1000

    for object_left in objects_left:
        object_left_distance = np.sqrt((object_left.x-pose.x)**2 + (object_left.y-pose.y)**2)
        if object_left_distance < min_distance:
            next_object = object_left
            min_distance = object_left_distance

    print(next_object)

    return next_object


def trajectory_until_object(pose, next_object):

    start = pose
    goal = Pose(next_object.x, next_object.y, 0)
    distance = np.sqrt((goal.x-start.x)**2 + (goal.y-start.y)**2)
    num_discretization_step = round(distance*10)

    x_in_trajectory = np.linspace(start.x, goal.x, num_discretization_step)
    y_in_trajectory = np.linspace(start.y, goal.y, num_discretization_step)
    angle_in_trajectory = np.tanh((goal.y-start.y)/(goal.x-start.x))

    planned_trajectory = []

    for i in num_discretization_step:
        planned_trajectory.append(Pose(x_in_trajectory[i], y_in_trajectory[i], angle_in_trajectory[i]))

    print(planned_trajectory)

    return planned_trajectory


def trajectory_around_object(pose, next_object):

    start = pose
    radius = 0.3
    distance = 2*np.pi*radius
    num_discretization_step = round(distance*10)
    planned_trajectory = []

    for i in range(0, num_discretization_step):
        angle_in_trajectory = (2*np.pi/num_discretization_step)*i
        x_in_trajectory = start.x-(radius-np.cos(angle_in_trajectory))
        y_in_trajectory = start.y-(radius-np.sin(angle_in_trajectory))
        planned_trajectory.append(Pose(x_in_trajectory, y_in_trajectory, angle_in_trajectory))
    
    if next_object.color == "red":
        plan_trajectory = reversed(plan_trajectory)

    print(planned_trajectory)

    return planned_trajectory


def plan_trajectory(objects, current_pose):

    objects_left = objects
    pose = current_pose  
    planned_trajectory = [pose]
    count = 0

    while objects_left != []:

        next_object = select_next(pose, objects_left)

        next_trajectory = trajectory_until_object(pose, next_object)
        planned_trajectory.extend(next_trajectory)

        next_trajectory = trajectory_around_object(pose, next_object)
        planned_trajectory.extend(next_trajectory)  

        objects_left = objects_left.remove(next_object)

        count = count + 1        
        if count > 10000: 
            print("Error in planning the trajectory")
            break

    print(planned_trajectory)

    return planned_trajectory


def drive_trajectory(objects, current_pose):

    planned_trajectory = plan_trajectory(objects, current_pose)

    x = current_pose.x
    y = current_pose.y
    angle = current_pose.angle

    for waypoint in planned_trajectory:

        dest_x = waypoint.x
        dest_y = waypoint.y

        moveRobot(x, y, angle, dest_x, dest_y)
        current_pose = get_pose()

        x = current_pose.x
        y = current_pose.y
        angle = current_pose.angle


current_pose = Pose(0, 0, 0)

objects = [Object("cube", 1, 1, "blue"), Object("cube", 2, 1, "red"), Object("cube", 2, 3, "blue")]

drive_trajectory(objects, current_pose)
