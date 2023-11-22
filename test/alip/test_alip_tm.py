import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

script_directory = os.path.dirname(os.path.abspath(__file__))
file_path1 = os.path.join(script_directory, 'alip_COM_trajectory.txt')

print(script_directory)
print(file_path1)
# Read data from the .txt file


def all_trajectories(file_path):
    trajectories = []
    current_trajectory = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('end'):
                print(file_path, current_trajectory , ' ','\n \n')
                if current_trajectory:
                    trajectories.append(np.array(current_trajectory))
                else:
                    trajectories.append(np.array([0,0,0,0]))
            elif line.startswith('start'):
                current_trajectory = []
            else:
                values = [float(val) for val in line.split()]
                current_trajectory.append(values)
    
    return trajectories

plotfreq = 1

def plot_trajectories(trajectories, file_path):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    counter = 0;
    
    for trajectory in trajectories:
        if counter % plotfreq == 0:
            x = trajectory[:, 0]
            y = trajectory[:, 1]
            z = trajectory[:, 2]
            time = trajectory[:, 3]
            ax.plot(x, y, z, marker='o', linestyle='-', label=f'Trajectory {counter}')
            ax.plot(x[0],y[0], z[0], marker='x', color='r')
            ax.plot(x[-1], y[-1], z[-1], marker = 'x', color='y')
        counter +=1

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(file_path)
    ax.legend()


def merge_swing_traj(traj1, traj2):
    merged_trajectories = []
    for i in range(len(traj2)):
        #merged_trajectorie = traj1[i]
        #np.append(merged_trajectorie, traj2[i])
        #merged_trajectorie.append(traj2[i])
        #np.concatenate(merged_trajectorie, traj2[i])
        #merged_trajectories.append(merged_trajectorie)
        merged_trajectorie = np.concatenate((traj1[i], traj2[i]))
        merged_trajectories.append(merged_trajectorie)

    return merged_trajectories




trCOM = all_trajectories('alip_COM_trajectory.txt')
trSw1 = all_trajectories('Swing1_trajectory.txt')
trSw2 = all_trajectories('Swing2_trajectory.txt')

trBezier = all_trajectories('BezierSwing_trajectory.txt')
trAlip = all_trajectories('AlipSwing_trajectory.txt')
trAlip2 = all_trajectories('Alip2Swing_trajectory.txt')


trSw = merge_swing_traj(trSw1, trSw2)




#plot_trajectories(trCOM, 'alip_COM_trajectory.txt')
#plot_trajectories(trSw1, 'Swing1_trajectory')
#plot_trajectories(trSw2, 'Swing2_trajectory')


plot_trajectories(trSw, 'full Swing')
plot_trajectories(trBezier, 'Quadratic Bezier')
plot_trajectories(trAlip, 'Alip Swing')
plot_trajectories(trCOM, 'com')
plot_trajectories(trAlip2, ' Alip paper 2')

plt.show()



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
counter = 0;

for trajectory in trCOM:
    if counter % plotfreq == 0:
        x = trajectory[:, 0]
        y = trajectory[:, 1]
        z = np.zeros(np.size(y))
        time = trajectory[:, 3]
        ax.plot(x, y, z, marker='x', linestyle='-', label=f'Trajectory {counter}')
        ax.plot(x[0],y[0], z[0], marker='x', color='r')
        ax.plot(x[-1], y[-1], z[-1], marker = 'x', color='y')
    counter +=1

for trajectory in trSw:
    if counter % plotfreq == 0:
        x = trajectory[:, 0]
        y = trajectory[:, 1]
        z = trajectory[:, 2]
        time = trajectory[:, 3]
        ax.plot(x, y, z, marker='o', linestyle='-', label=f'Trajectory {counter}')
        ax.plot(x[0],y[0], z[0], marker='x', color='r')
        ax.plot(x[-1], y[-1], z[-1], marker = 'x', color='y')
    counter +=1

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('ff')
ax.legend()


plt.show()
