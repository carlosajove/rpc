import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

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

def readRobotSwTr(file_path):
    trajectories = []
    with open(file_path, 'r') as file:
        for line in file:
            values = [float(val) for val in line.split()]
            trajectories.append(values)  
    trajectories = np.array(trajectories)  
    return trajectories

def plot_trajectories_with_robfoot(trajectories, rtraj, file_path):
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
    ax.plot(rtraj[:,0], rtraj[:,1], rtraj[:,2], marker = 'x')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(file_path)
    ax.legend()

def normal3Dplot(x, y, z, name):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z, marker='o', linestyle='-')
    ax.plot(x[0],y[0], z[0], marker='x', color='r')
    ax.plot(x[-1], y[-1], z[-1], marker = 'x', color='y')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(name)


trCOM = all_trajectories('alip_COM_trajectory.txt')
#trSw1 = all_trajectories('Swing1_trajectory.txt')
#trSw2 = all_trajectories('Swing2_trajectory.txt')

#trBezier = all_trajectories('BezierSwing_trajectory.txt')
trAlip = all_trajectories('AlipSwing_trajectory.txt')
trAlip2 = all_trajectories('Alip2Swing_trajectory.txt')
trRobotSwing = readRobotSwTr('robotSwingFootTraj.txt')
#trSw = merge_swing_traj(trSw1, trSw2)

CurrentComstate = readRobotSwTr('RobotCOM.txt')
RobotCommand = readRobotSwTr('RobotCommand.txt')
MpcComState = readRobotSwTr('MpcCOMstate.txt')



with open('LandTime.txt', 'r') as file:
    landingTime = [float(line.strip()) for line in file]

#plot_trajectories(trCOM, 'alip_COM_trajectory.txt')
#plot_trajectories(trSw1, 'Swing1_trajectory')
#plot_trajectories(trSw2, 'Swing2_trajectory')


#plot_trajectories(trSw, 'full Swing')
#plot_trajectories(trBezier, 'Quadratic Bezier')
#plot_trajectories(trAlip, 'Alip Swing')
#plot_trajectories(trCOM, 'com')
#plot_trajectories(trAlip2, ' Alip paper 2')
#plot_trajectories_with_robfoot(trAlip2, trRobotSwing, ' Alip paper 2 tracking')
#plot_trajectories_with_robfoot(trAlip, trRobotSwing, 'Alip Swing ')
#plt.show()





#COM at change swing foot
xCOMstance = CurrentComstate[:,0]
yCOMstance = CurrentComstate[:,1]
zCOMstance = CurrentComstate[:,2]
LxCOM= CurrentComstate[:,3]
LyCOM = CurrentComstate[:,4]
LzCOM = CurrentComstate[:,5]
timeCOM = CurrentComstate[:,6]
xCOMworld = CurrentComstate[:,7]
yCOMworld = CurrentComstate[:,8]
zCOMworld = CurrentComstate[:,9]
stleg_xWorld = CurrentComstate[:,10]
stleg_yWorld = CurrentComstate[:,11]
stleg_zWorld = CurrentComstate[:,12]



MpcxCOM = MpcComState[:,0]
MpcyCOM = MpcComState[:,1]
MpczCOM = MpcComState[:,2]
MpcLxCOM= MpcComState[:,3]
MpcLyCOM = MpcComState[:,4]
MpcLzCOM = MpcComState[:,5]
MpctimeCOM = MpcComState[:,6]
MpcLx_off = MpcComState[:,7]
MpcLy_des = MpcComState[:,8]
MpcMu = MpcComState[:,9]
MpcLegWidth = MpcComState[:,10]
MpczH = MpcComState[:,11]
MpcMassCom = MpcComState[:,12]
MpcTs = MpcComState[:,13]

swingXCommand = RobotCommand[:,0]
swingYCommand = RobotCommand[:,1]
swingZCommand = RobotCommand[:,2]

#Compute desired state x,y,Lx:
l = np.sqrt(9.81/MpczH)
Lx_plus = 0.5*MpcMassCom*MpczH*MpcLegWidth*l*np.tanh(0.5*MpcTs*l) + MpcLx_off
Lx_minus = -0.5*MpcMassCom*MpczH*MpcLegWidth*l*np.tanh(0.5*MpcTs*l) + MpcLx_off


plt.figure()
plt.plot(timeCOM, LyCOM)
plt.plot(MpctimeCOM, MpcLyCOM)
plt.title('robot Ly  ')
for t in landingTime:
    plt.axvline(x=t, color='black', linestyle='-', linewidth=0.5)

plt.figure()
plt.plot(timeCOM, LxCOM)
plt.plot(MpctimeCOM, MpcLxCOM)
plt.title('robot Lx ')
for t in landingTime:
    plt.axvline(x=t, color='black', linestyle='-', linewidth=0.5)


plt.figure()
plt.plot(timeCOM, LzCOM)
plt.plot(MpctimeCOM, MpcLzCOM)
plt.title('robot Lz ')
for t in landingTime:
    plt.axvline(x=t, color='black', linestyle='-', linewidth=0.5)




#normal3Dplot(xCOM, yCOM, zCOM, 'COM st ref')

#normal3Dplot(RobotCommand[:,6], RobotCommand[:,7], RobotCommand[:,8], 'Foot Acceleration')


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(swingXCommand, swingYCommand, swingZCommand, marker='x', label = 'Command')
ax.plot(trRobotSwing[:,0], trRobotSwing[:,1], trRobotSwing[:,2], marker = 'x', color = 'red', label = 'pos')
ax.legend()


plt.figure()
plt.plot(swingXCommand, trRobotSwing[:,0])
plt.title('x Command vs Swing')

plt.figure()
plt.scatter(timeCOM, trRobotSwing[:,0], label = 'pos')
plt.scatter(timeCOM, swingXCommand, label =  'command')
plt.title('time vs x')
plt.legend()


plt.figure()
plt.plot(swingYCommand, trRobotSwing[:,1])
plt.title('y Command vs Swing')

plt.figure()
plt.scatter(timeCOM, trRobotSwing[:,1], label = 'pos')
plt.scatter(timeCOM, swingYCommand, label =  'command')
plt.title('time vs y')
plt.legend()

plt.figure()
plt.plot(swingZCommand, trRobotSwing[:,2])
plt.title('z Command vs Swing')

plt.figure()
plt.scatter(timeCOM, trRobotSwing[:,2], label = 'pos')
plt.scatter(timeCOM, swingZCommand, label =  'command')
plt.title('time vs Z')
plt.legend()


#alip paper plots
#compute desired state
lbound_time = 10
ubound_time = 75


if lbound_time not in timeCOM or ubound_time not in timeCOM:
    print("Error: lbound_time or ubound_time not found in timeCOM vector.")
    print("min time: ", timeCOM[0])
    print("max time: " , timeCOM[-1])
    plt.show()
    exit()
    
else:
    lbound_idx = np.searchsorted(timeCOM, lbound_time, side='left')
    ubound_idx = np.searchsorted(timeCOM, ubound_time, side='right')

inter = slice(lbound_idx, ubound_idx)

landingTimes_in_range = [t for t in landingTime if lbound_time <= t <= ubound_time]


#normal3Dplot(xCOMworld[inter], yCOMworld[inter], zCOMworld[inter], 'COM world ref')


# Extracting a subset of data within the specified time range
x = timeCOM[inter]
y = LxCOM[inter]/(MpcMassCom[inter]*MpczH[inter])
yy = Lx_plus[inter]/(MpcMassCom[inter]*MpczH[inter])
yyy = Lx_minus[inter]/(MpcMassCom[inter]*MpczH[inter])
plt.figure()
plt.plot(x, y)
#plt.plot(MpctimeCOM, MpcLxCOM/(MpcMassCom*MpczH))
plt.plot(x,yy , label='Lx_plus')
plt.plot(x, yyy, label='Lx_minus')
for t in landingTimes_in_range:
    plt.axvline(x=t, color='black', linestyle='-', linewidth=0.1)

plt.title('Lx/mzH paper plot')

y = LyCOM[inter]/(MpcMassCom[inter]*MpczH[inter])
ydes = MpcLy_des[inter]/(MpcMassCom[inter]*MpczH[inter])

plt.figure()
plt.plot(x, y, label = 'robot')
plt.plot(x, ydes, label = 'des')
#plt.plot(MpctimeCOM, MpcLyCOM/(MpcMassCom*MpczH))
for t in landingTimes_in_range:
    plt.axvline(x=t, color='black', linestyle='-', linewidth=0.1)
plt.title('Ly/mzH paper plot')
plt.legend()


y = xCOMworld[inter]
plt.figure()
plt.plot(x, y)
plt.title('COM x world')

y = yCOMworld[inter]
plt.figure()
plt.plot(x, y)
plt.title('COM y world')

y = zCOMworld[inter]
plt.figure()
plt.plot(x, y)
plt.title('COM z world')


#stance has discontinuities at change of stance

x = timeCOM[inter]
y = zCOMstance[inter]

plt.figure()
plt.plot(x, y)
plt.title('COM z')
for t in landingTime:
    plt.axvline(x=t, color='black', linestyle='-', linewidth=0.1)


x = timeCOM[inter]
y = yCOMstance[inter]
discontinuity_indices = np.where(np.abs(np.diff(y)) > 0.05)[0]
x[discontinuity_indices] = np.nan
y[discontinuity_indices] = np.nan

plt.figure()
plt.plot(x, y)
plt.title('COM y')
for t in landingTime:
    plt.axvline(x=t, color='black', linestyle='-', linewidth=0.1)



x = timeCOM[inter]
y = xCOMstance[inter]
discontinuity_indices = np.where(np.abs(np.diff(y)) > 0.05)[0]
x[discontinuity_indices] = np.nan
y[discontinuity_indices] = np.nan

plt.figure()
plt.plot(x, y)
plt.title('COM x')
for t in landingTime:
    plt.axvline(x=t, color='black', linestyle='-', linewidth=0.1)





plt.show()  
