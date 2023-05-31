import numpy as np


class Object:

    def __init__(self, form, x, y, color) -> None:
        # static variables
        self.form: str = form
        self.x: float = x
        self.y: float = y
        self.color: str = color


class Pose:

    def __init__(self, x, y, angle) -> None:
        # static variables
        self.x: float = x
        self.y: float = y
        self.angle: float = angle


def select_next(pose, objects_left) -> Object:

    next_object: Object
    min_distance = 1000

    for object_left in objects_left:
        object_left_distance = np.sqrt((object_left.x-pose.x)**2 + (object_left.y-pose.y)**2)
        if object_left_distance < min_distance:
            next_object = object_left

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

    return planned_trajectory


def plan_trajectory(objects):

    objects_left = objects
    initial_pose = Pose(0, 0, 0)  
    planned_trajectory = [initial_pose]
    pose = initial_pose

    while objects_left != []:

        next_object = select_next(pose, objects_left)

        next_trajectory = trajectory_until_object(pose, next_object)
        planned_trajectory.append(next_trajectory)

        next_trajectory = trajectory_around_object(pose, next_object)
        planned_trajectory.append(next_trajectory)  

        objects_left = objects_left.remove(next_object)

    return planned_trajectory