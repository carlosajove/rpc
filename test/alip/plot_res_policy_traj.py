import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import scipy
from scipy.linalg import expm
from plot_utils import *

matplotlib.use('TkAgg')
import math
from read_data_res_policy import *

Lydes_m = MpcLy_des[inter] / (MpcMassCom[inter] * MpczH[inter])
###########################
## Plot residual action, mpc solution, full action
###########################
fig, axes = plt.subplots(1, 3)
axes[0].plot(action_time, residual_action_x, color='r', label='res action')
axes[0].plot(action_time, mpc_sol_in_world_x, color='b', label='mpc sol')
axes[0].plot(action_time, full_action_x, color='k', label='full sol')
axes[0].plot(time[inter], Lydes_m, label=r'$L_{y}^{des}$')
axes[0].grid(True)

axes[1].plot(action_time, residual_action_y, color='r', label='res action')
axes[1].plot(action_time, mpc_sol_in_world_y, color='b', label='mpc sol')
axes[1].plot(action_time, full_action_y, color='k', label='full sol')
axes[1].plot(time[inter], Lydes_m, label=r'$L_{y}^{des}$')
axes[1].grid(True)

axes[2].plot(action_time, residual_action_yaw, color='r', label='res action')
# axes[2].plot(action_time, mpc_sol_in_world_y, color='k', label='mpc sol')
axes[2].plot(action_time, full_action_yaw, color='k', label='full sol')
axes[2].plot(time[inter], Lydes_m, label=r'$L_{y}^{des}$')
axes[2].grid(True)

###########################
## Plot residual action, mpc solution, full action (version 2)
###########################
plt.figure(figsize=(10, 6))
plt.plot(action_time, residual_action_x, color='r', label=r'$\Delta_x$')
plt.plot(action_time, residual_action_y, color='g', label=r'$\Delta_y$')
plt.plot(action_time,
         residual_action_yaw,
         color='b',
         label=r'$\Delta_{\textrm{yaw}}$')
plt.plot(time[inter], Lydes_m, color='k', label=r'$L_{y}^{des}$')
plt.grid(True)

plt.show()
