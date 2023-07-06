import numpy as np
#from scripts.pose import get_pose, Pose
#from scripts.motion import drive_trajectory
import matplotlib.pyplot as plt

class Pose:

    def __init__(self, x, y, angle) -> None:
        self.x: float = x
        self.y: float = y
        self.angle: float = angle

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

def plan_trajectory(num_discretization_step):
    
    planned_trajectory = []

    max_x = 1.485
    discretization_step = max_x/num_discretization_step
    angle_in_trajectory = 0

    first_x = 0
    last_x = num_discretization_step
    direction = 1

    for j in range(0,num_discretization_step):
        
        for i in range(first_x,last_x,direction):
            x_in_trajectory = (i)*discretization_step + 0.05
            y_in_trajectory = (j)*discretization_step + 0.05
            planned_trajectory.append(Pose(x_in_trajectory, y_in_trajectory, angle_in_trajectory))
        print(first_x, last_x)
        direction *= -1
        first_x, last_x = last_x + direction, first_x + direction
        angle_in_trajectory = (angle_in_trajectory+np.pi)%(2*np.pi)
    planned_trajectory.append(Pose(0.05, 0.05, 0))

    plot_trajectory(planned_trajectory)

    return planned_trajectory

if __name__ == '__main__':
    #current_pose = get_pose()
    num_discretization_step = 15
    planned_trajectory = plan_trajectory(num_discretization_step)
    #drive_trajectory(planned_trajectory)



