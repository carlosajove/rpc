import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import scipy
from scipy.linalg import expm
from plot_utils import *

matplotlib.use('TkAgg')
import math
from read_data_all import *

from matplotlib.animation import FFMpegWriter

plt.rcParams['animation.ffmpeg_path'] = 'ffmpeg'

########################
# Config
########################
b_make_live_plot = True

#########################
#Tracking desired Lx, Ly
#########################
# _, _, _, _, _, _, st, _ = read_task('task_com_z.txt', "com_z")
# st = st[inter]
# st = st[:-2]

###########################
#Ly trakcing
###########################
Ly_m = (mpc_coor_L_y[inter]) / (MpcMassCom[inter] * MpczH[inter])
Ly_m_mpc_rl = (mpc_coor_L_y_mpc_rl[inter_mpc_rl]) / (
    MpcMassCom_mpc_rl[inter_mpc_rl] * MpczH_mpc_rl[inter_mpc_rl])
Lydes_m = MpcLy_des[inter] / (MpcMassCom[inter] * MpczH[inter])
Lydes_m_mpc_rl = MpcLy_des_mpc_rl[inter_mpc_rl] / (
    MpcMassCom_mpc_rl[inter_mpc_rl] * MpczH_mpc_rl[inter_mpc_rl])
plt.figure(figsize=(8, 6))
# plt.plot(time[inter] + 1.0, Lydes_m, label=r'$L_{y}^{des,mpc}$', color='r')
# plt.plot(time[inter] + 1.0, Ly_m, label=r'$L_{y}^{mpc}$', color='b')
plt.plot(time[inter] + 1.0, Ly_m, label=r'$L_{y}^{mpc}$', color='b')
plt.plot(time_mpc_rl[inter_mpc_rl],
         Lydes_m_mpc_rl,
         label=r'$L_{y}^{des, rl}$',
         color='k')
plt.plot(time_mpc_rl[inter_mpc_rl],
         Ly_m_mpc_rl,
         label=r'$L_{y}^{rl}$',
         color='orange')
plt.xlabel('Time', fontsize=20)
plt.ylabel(r'$\frac{L_{y}}{mz_{H}}$',
           fontsize=20,
           rotation='horizontal',
           labelpad=20)
plt.legend(fontsize=16)
# for i in range(len(landingTimes_in_range) - 1):
# plt.axvspan(landingTimes_in_range[i] + 1,
# landingTimes_in_range[i + 1] + 1,
# color=colors[i % len(colors)],
# alpha=0.3)
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.xlim(5, 65)
plt.ylim(-1, 1)
plt.grid(True)

###########################
#Lx_des
###########################
# Lx_m = (mpc_coor_L_x[inter]) / (MpcMassCom[inter] * MpczH[inter])
# Lxdes_m1 = Lx_plus[inter] / (MpcMassCom[inter] * MpczH[inter])
# Lxdes_m2 = Lx_minus[inter] / (MpcMassCom[inter] * MpczH[inter])
# plt.figure(figsize=(10, 6))
# plt.plot(time[inter], Lx_m, label=r'$L_{x}$')
# plt.plot(time[inter], Lxdes_m1, label=r'$L_{x}^{des,p}$')
# plt.plot(time[inter], Lxdes_m2, label=r'$L_{x}^{des,m}$')
# plt.xlabel('Time (s)', fontsize=16)
# plt.ylabel(r'$\frac{L_{x}}{mz_{H}}$',
# rotation='horizontal',
# fontsize=20,
# labelpad=20)
# plt.legend(fontsize=16)
# for i in range(len(landingTimes_in_range) - 1):
# plt.axvspan(landingTimes_in_range[i] + 1,
# landingTimes_in_range[i + 1] + 1,
# color=colors[i % len(colors)],
# alpha=0.3)
# plt.xticks(fontsize=14)
# plt.yticks(fontsize=14)
#plt.ylim(-1, 1)

###########################
# Tracking desired yaw
###########################
# plt.figure(figsize=(10, 6))
# plt.plot(time[inter], torso_yaw[inter])
# plt.xlabel('Time (s)', fontsize=16)
# plt.ylabel('torso yaw (rad)', fontsize=16)
# for i in range(len(landingTimes_in_range) - 1):
# plt.axvspan(landingTimes_in_range[i] + 1,
# landingTimes_in_range[i + 1] + 1,
# color=colors[i % len(colors)],
# alpha=0.3)
# plt.xticks(fontsize=14)
# plt.yticks(fontsize=14)
# plt.ylim(-math.pi, math.pi)

################################
# make LIVE plots
################################
if b_make_live_plot:
    fig = plt.figure(figsize=(8.5, 5))

    des_plot = plt.plot(time_mpc_rl[inter_mpc_rl][1000:],
                        Lydes_m_mpc_rl[1000:],
                        'r--',
                        label=r'$L_{y}^{des}$')
    mpc_rl_actual_plot, = plt.plot([], [], color='orange', label='MPC+RL')
    mpc_actual_plot, = plt.plot([], [], color='b', label='MPC')

    # plt.xlim(np.min(time[inter], np.max(time[inter])))
    plt.xlim(np.min(time[inter]), np.max(time[inter]))
    plt.ylim(-1, 1)
    plt.xlabel('Time (s)', fontsize=16)
    plt.ylabel(r'$\frac{L_{y}}{mz_{H}} (m/s)$', fontsize=16)
    plt.grid(True)
    plt.legend(loc="upper right", fontsize=16)

    xlist = []
    xlist_mpc_rl = []
    # des_list = []
    act_mpc_rl_list = []
    act_mpc_list = []

    # writer = FFMpegWriter(fps=24)
    writer = FFMpegWriter(fps=574)
    with writer.saving(fig, "mpc_freq_Ly.mp4", 100):
        for i, j in zip(range(len(time[inter])),
                        range(len(time_mpc_rl[inter_mpc_rl]))):
            xlist.append(time[inter][i + 1000] + 1.0)
            xlist_mpc_rl.append(time_mpc_rl[inter_mpc_rl][i + 1000])
            act_mpc_rl_list.append(Ly_m_mpc_rl[i + 1000])
            act_mpc_list.append(Ly_m[i + 1000])

            mpc_rl_actual_plot.set_data(xlist_mpc_rl, act_mpc_rl_list)
            mpc_actual_plot.set_data(xlist, act_mpc_list)

            writer.grab_frame()

plt.show()
