import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

script_directory = os.path.dirname(os.path.abspath(__file__))
file_path1 = os.path.join(script_directory, 'alip_COM_trajectory.txt')

print(script_directory)
print(file_path1)
# Read data from the .txt file

"""
with open('alip_COM_trajectory.txt', 'r') as file:
    lines = file.readlines()

# Extracting data from the lines
x, y, z, time = [], [], [], []
for line in lines:
    values = line.split()
    x.append(float(values[0]))
    y.append(float(values[1]))
    z.append(float(values[2]))
    time.append(float(values[3]))

# Creating a 3D trajectory plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(x, y, z, marker='o', linestyle='-', color='b')
ax.plot(x[0],y[0], marker='x', color='r')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title(f'Trajectory Plot (Time: {time[0]} to {time[-1]})')



with open('Swing1_trajectory.txt', 'r') as file2:
    lines2 = file2.readlines()

with open('Swing2_trajectory.txt', 'r') as file3:
    lines3 = file3.readlines()

x, y, z, time = [], [], [], []
for line in lines2:
    values = line.split()
    x.append(float(values[0]))
    y.append(float(values[1]))
    z.append(float(values[2]))
    time.append(float(values[3]))

for line in lines3:
    values = line.split()
    x.append(float(values[0]))
    y.append(float(values[1]))
    z.append(float(values[2]))
    time.append(0.99+float(values[3]))

print(x)


fig1 = plt.figure()
bx = fig1.add_subplot(111, projection='3d')
bx.plot(x, y, z, marker='o', linestyle='-', color='b')
bx.plot(x[0],y[0], marker='x', color='r')
bx.plot(x[-1], y[-1], marker = 'x', color='y')
bx.set_xlabel('X')
bx.set_ylabel('Y')
bx.set_zlabel('Z')
bx.set_title(f'Swing 2 (Time: {time[0]} to {time[-1]})')





with open('BezierSwing_trajectory.txt', 'r') as file4:
    lines4 = file4.readlines()

x, y, z, time = [], [], [], []
for line in lines4:
    values = line.split()
    x.append(float(values[0]))
    y.append(float(values[1]))
    z.append(float(values[2]))
    time.append(float(values[3]))


fig2 = plt.figure()
cx = fig2.add_subplot(111, projection='3d')
cx.plot(x, y, z, marker='o', linestyle='-', color='b')
cx.plot(x[0],y[0], marker='x', color='r')
cx.plot(x[-1], y[-1], marker = 'x', color='y')
cx.set_xlabel('X')
cx.set_ylabel('Y')
cx.set_zlabel('Z')
cx.set_title(f'Swing 2 (Time: {time[0]} to {time[-1]})')


plt.show()
"""

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
    plt.show()


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




"""
        ax.plot(x, y, label=f'Time: {time[0]} to {time[-1]}')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(file_path)
    ax.legend()
    plt.show()
"""
trCOM = all_trajectories('alip_COM_trajectory.txt')
trSw1 = all_trajectories('Swing1_trajectory.txt')
trSw2 = all_trajectories('Swing2_trajectory.txt')

trSw = merge_swing_traj(trSw1, trSw2)




plot_trajectories(trCOM, 'alip_COM_trajectory.txt')
#plot_trajectories(trSw1, 'Swing1_trajectory')
#plot_trajectories(trSw2, 'Swing2_trajectory')


plot_trajectories(trSw, 'full Swing')

fig4 = plt.figure()
for trajectory in trCOM:
    plt.plot(trajectory[:,2], trajectory[:,3])
    plt.show()