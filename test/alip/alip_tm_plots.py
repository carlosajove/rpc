import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import scipy
from scipy.linalg import expm
from plot_utils import *
matplotlib.use('TkAgg')

lbound_time = 4
ubound_time = 25

script_directory = os.path.dirname(os.path.abspath(__file__))

print(script_directory)

trAlip2 = all_trajectories('Alip2Swing_trajectory.txt')
trRobotSwing = readRobotSwTr('robotSwingFootTraj.txt')
#trSw = merge_swing_traj(trSw1, trSw2)

CurrentComstate = readRobotSwTr('RobotCOM.txt')
RobotCommand = readRobotSwTr('RobotCommand.txt')
MpcComState = readRobotSwTr('MpcCOMstate.txt')

COMmpcoor = readRobotSwTr('RobotCOMmpcOri.txt')


mpc_coor_x = COMmpcoor[:, 0]
mpc_coor_y = COMmpcoor[:, 1]
mpc_coor_z = COMmpcoor[:, 2]
mpc_coor_vx = COMmpcoor[:, 3]
mpc_coor_vy = COMmpcoor[:, 4]
mpc_coor_vz = COMmpcoor[:, 5]
mpc_coor_Lst_x = COMmpcoor[:, 6]
mpc_coor_Lst_y = COMmpcoor[:, 7]
mpc_coor_Lst_z = COMmpcoor[:, 8]
mpc_coor_Lc_x = COMmpcoor[:, 9]
mpc_coor_Lc_y = COMmpcoor[:, 10]
mpc_coor_Lc_z = COMmpcoor[:, 11]
mpc_coor_L_x = COMmpcoor[:,12]
mpc_coor_L_y = COMmpcoor[:,13]
mpc_coor_L_z = COMmpcoor[:,14]
alip_time = COMmpcoor[:,15]

timeCOM = alip_time
time = alip_time

swingXCommand = RobotCommand[:,0]
swingYCommand = RobotCommand[:,1]
swingZCommand = RobotCommand[:,2]
swingVxCommand = RobotCommand[:, 3]
swingVyCommand = RobotCommand[:, 4]
swingVzCommand = RobotCommand[:, 5]
swingAxCommand = RobotCommand[:, 6]
swingAyCommand = RobotCommand[:, 7]
swingAzCommand = RobotCommand[:, 8]
swingTimeCommand = RobotCommand[:,9]
end_foot_commandx = RobotCommand[:, 10]
end_foot_commandy = RobotCommand[:, 11]
end_foot_commandz = RobotCommand[:, 12]

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

with open('LandTime.txt', 'r') as file:
    landingTime = [float(line.strip()) for line in file]


inter = sliceTime(time, lbound_time, ubound_time)
landingTimes_in_range = [t for t in landingTime if lbound_time <= t <= ubound_time]

############################
## Swing Foot 3d tracking ##
############################
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(swingXCommand, swingYCommand, swingZCommand, marker='x', label = 'Command')
ax.plot(trRobotSwing[:,0], trRobotSwing[:,1], trRobotSwing[:,2], marker = 'x', color = 'red', label = 'pos')
ax.legend()


############################
##  Tracking of desired   ##
############################


# MPC evolution vs real L
l = np.sqrt(9.81/MpczH)
Lx_plus = 0.5*MpcMassCom*MpczH*MpcLegWidth*l*np.tanh(0.5*MpcTs*l) + MpcLx_off
Lx_minus = -0.5*MpcMassCom*MpczH*MpcLegWidth*l*np.tanh(0.5*MpcTs*l) + MpcLx_off

plt.figure()
plt.plot(alip_time, mpc_coor_L_y)
plt.plot(MpctimeCOM, MpcLyCOM)
plt.title('robot Ly  ')
for t in landingTime:

    plt.axvline(x=t, color='black', linestyle='-', linewidth=0.5)


plt.figure()
plt.plot(alip_time, mpc_coor_L_x)
plt.plot(MpctimeCOM, MpcLxCOM)
plt.title('robot Lx ')
for t in landingTime:
    plt.axvline(x=t, color='black', linestyle='-', linewidth=0.5)

plt.figure()
plt.plot(alip_time, mpc_coor_L_z)
plt.plot(MpctimeCOM, MpcLzCOM)
plt.title('robot Lz ')
for t in landingTime:
    plt.axvline(x=t, color='black', linestyle='-', linewidth=0.5)


#Tracking desired Lx, Ly
light_grey = [0.85, 0.85, 0.85]
facecolors = [
    light_grey, 'grey', 'brown', 'red', 'orange', 'yellow', 'green', 'blue', 'purple',
    'crimson', 'white'
] 
colors =[facecolors[0], facecolors[-1]]
_, _, _, _, _, _, st, _ = read_task('task_com_z.txt', "com_z")
st = st[inter]
st = st[:-2]

#Ly des
Ly_m = (mpc_coor_L_y[inter])/(MpcMassCom[inter]*MpczH[inter])
Lydes_m = MpcLy_des[inter]/(MpcMassCom[inter]*MpczH[inter])
plt.figure(figsize=(10, 6))
plt.plot(time[inter], Ly_m, label=r'$L_{y}$')
plt.plot(time[inter], Lydes_m, label=r'$L_{y}^{des}$')
plt.xlabel('Time', fontsize = 20)
plt.ylabel(r'$\frac{L_{y}}{mz_{H}}$', fontsize = 20, rotation='horizontal', labelpad=20)
plt.legend(fontsize=16)
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
plt.xticks(fontsize=14) 
plt.yticks(fontsize=14) 

#Lx_des
Lx_m = (mpc_coor_L_x[inter])/(MpcMassCom[inter]*MpczH[inter])
Lxdes_m1 = Lx_plus[inter]/(MpcMassCom[inter]*MpczH[inter])
Lxdes_m2 = Lx_minus[inter]/(MpcMassCom[inter]*MpczH[inter])
plt.figure(figsize=(10, 6))
plt.plot(time[inter], Lx_m, label = r'$L_{x}$')
plt.plot(time[inter], Lxdes_m1, label = r'$L_{x}^{des,p}$')
plt.plot(time[inter], Lxdes_m2, label = r'$L_{x}^{des,m}$')
plt.xlabel('Time (s)', fontsize = 16)
plt.ylabel(r'$\frac{L_{x}}{mz_{H}} (ms^{-1})$', fontsize = 20, rotation='horizontal', labelpad=20)
plt.legend(fontsize=16)
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
plt.xticks(fontsize=14) 
plt.yticks(fontsize=14) 





#End foot command
plt.figure()
plt.scatter(end_foot_commandx[inter], end_foot_commandy[inter], marker = 'x')





#########################################################
## Plots ALIP continuous trajectory vs real trajectory ##
#########################################################
def alip_trajectory(x0, dt): #x = [x y Lx Ly]
    mass = 39.15342
    zH = 0.69
    g = 9.81
    A = np.array([[ 0, 0 , 0, 1/(mass*zH)],
                 [0, 0, -1/(mass*zH), 0],
                 [0, -mass*g, 0, 0],
                 [mass*g , 0, 0, 0]])
    Adt = expm(A*dt) 
    res = np.matmul(Adt, x0)
    return res

traj_indices = np.where(np.isin(time, landingTimes_in_range))[0]
alip_state_traj = []
alip_time_traj = []
for i in range(np.size(traj_indices)-2):
    x0 = np.array([MpcxCOM[traj_indices[i]+4],
                   MpcyCOM[traj_indices[i]+4],
                   MpcLxCOM[traj_indices[i]+4],
                   MpcLyCOM[traj_indices[i]+4]])
    for j in range(traj_indices[i]+5, traj_indices[i+1]):
        dt = time[j] - time[traj_indices[i]]
        res = alip_trajectory(x0, dt)
        alip_state_traj.append(res)
        alip_time_traj.append(time[j])

alip_state_traj = np.stack(alip_state_traj)
alip_time_traj = np.array(alip_time_traj)
"""
def plot_alip_traj_vs_real(dim, mpc_coor_, name):
    plt.figure()
    plt.plot(time[traj_indices[0]+5:traj_indices[-2]], mpc_coor_[traj_indices[0]+5:traj_indices[-2]], label = "real " + name)
    plt.plot(alip_time_traj, alip_state_traj[:,dim], label = "alip " + name )
    for i in range(len(landingTimes_in_range) - 1):
        plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
    plt.legend()
"""

def plot_alip_traj_vs_real(dim, mpc_coor_, name_robot, name_pred, axis_y, eps1=1e-4, eps2=1e-4):
    plt.figure(figsize=(9, 6))
    #plt.plot(time[traj_indices[0]+5:traj_indices[-2]], mpc_coor_[traj_indices[0]+5:traj_indices[-2]], label="real " + name)
    real_traj_mask = np.abs(np.diff(mpc_coor_[traj_indices[0]:traj_indices[-2]], prepend=mpc_coor_[traj_indices[0]])) < eps1
    
    # Plot filtered real trajectory
    plt.plot(time[traj_indices[0]:traj_indices[-2]], np.where(real_traj_mask, mpc_coor_[traj_indices[0]:traj_indices[-2]], np.nan), label=name_robot)
    
    # Calculate mask for points with discontinuities greater than epsilon
    mask = np.abs(np.diff(alip_state_traj[:, dim], prepend=alip_state_traj[0, dim])) < eps2
    
    # Plot ALIP trajectory with discontinuities filtered
    alip_time_filtered = np.where(mask, alip_time_traj, np.nan)
    alip_state_filtered = np.where(mask, alip_state_traj[:, dim], np.nan)
    plt.plot(alip_time_traj, alip_state_filtered, label=name_pred)
    
    #plt.plot(alip_time_traj, alip_state_traj[:, dim], label="alip " + name)
    for i in range(len(landingTimes_in_range) - 1):
        plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
    plt.xlabel('Time (s)', fontsize = 12)
    plt.ylabel(axis_y, fontsize = 12, labelpad=20)
    plt.legend()

# Plot alip state
plot_alip_traj_vs_real(0, mpc_coor_x, name_robot="$x$", name_pred="$x^{pred}$", axis_y="$x \,(m)$", eps1 = 1e-3, eps2=  1e-3)
plot_alip_traj_vs_real(1, mpc_coor_y, name_robot="$y$", name_pred="$y^{pred}$", axis_y="$y \,(m)$", eps1= 1e-2, eps2=1e-2)
plot_alip_traj_vs_real(2, mpc_coor_L_x, name_robot="$L_{x}$", name_pred="$L_{x}^{pred}$", axis_y="$L_{x} \, (kg \dot m^{2} \dot s^{-1})$", eps1 = 1e-1, eps2 = 1e-1)
plot_alip_traj_vs_real(3, mpc_coor_L_y, name_robot="$L_{y}$", name_pred="$L_{y}^{pred}$", axis_y="$L_{y} \, (kg \dot m^{2} \dot s^{-1})$",eps1=0.075, eps2=0.05)

#neglecting L^x_c, L^y_c
plt.figure(figsize=(9, 6))
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_L_x[traj_indices[0]:traj_indices[-2]], label = '$L^{x}$')
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_Lc_x[traj_indices[0]:traj_indices[-2]], label = '$L^{x}_{c}$')
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
plt.xlabel('Time (s)', fontsize = 12)
plt.ylabel('$(kg \dot m^{2} \dot s^{-1})$', fontsize = 12, labelpad=20)
plt.legend()

plt.figure(figsize=(9, 6))
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_L_y[traj_indices[0]:traj_indices[-2]], label = '$L^{y}$')
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_Lc_y[traj_indices[0]:traj_indices[-2]], label = '$L^{y}_{c}$')
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
plt.xlabel('Time (s)', fontsize = 12)
plt.ylabel('$(kg \dot m^{2} \dot s^{-1})$', fontsize = 12, labelpad=20)
plt.legend()


plt.figure(figsize=(9, 6))
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_L_z[traj_indices[0]:traj_indices[-2]], label = '$L^{z}$')
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_Lc_z[traj_indices[0]:traj_indices[-2]], label = '$L^{z}_{c}$')
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
plt.xlabel('Time (s)', fontsize = 12)
plt.ylabel('$(kg \dot m^{2} \dot s^{-1})$', fontsize = 12, labelpad=20)
plt.legend()

plt.figure(figsize=(9, 6))
plt.plot(time[traj_indices[0]:traj_indices[-2]], mpc_coor_L_z[traj_indices[0]:traj_indices[-2]]-mpc_coor_Lc_z[traj_indices[0]:traj_indices[-2]], label = '$L^{z}-L^{z}_{c}$')
for i in range(len(landingTimes_in_range) - 1):
    plt.axvspan(landingTimes_in_range[i], landingTimes_in_range[i+1], color=colors[i % len(colors)], alpha=0.3)
plt.xlabel('Time (s)', fontsize = 12)
plt.ylabel('$(kg \dot m^{2} \dot s^{-1})$', fontsize = 12, labelpad=20)
plt.legend()

plt.show()








plt.show()





