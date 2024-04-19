import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from plot_utils import *


script_directory = os.path.dirname(os.path.abspath(__file__))

def plotTask(file, type, subtitle):
    pos, pos_d, vel, vel_d, weight, time, st, phase = read_task(file, type)
    plot_task(time, pos_d, pos, vel_d, vel, phase, subtitle)



z_pos, z_pos_d, z_vel, z_vel_d, z_weight, z_time, z_st, z_phase = read_task('task_com_z.txt', "com_z")

plot_task(z_time, z_pos_d, z_pos, z_vel_d, z_vel, z_phase, 'com_z')



plotTask('task_rf_ori.txt', "ori", "rfoot ori task")
plotTask('task_lf_ori.txt', "ori", "lfoot ori task")
plotTask('task_rf_pos.txt', "pos", "rfoot pos task")
plotTask('task_lf_pos.txt', "pos", "lfoot pos task")
plotTask('task_torso_com_ori.txt', "ori", "torso com ori")

u_pos, u_pos_d, u_vel, u_vel_des, u_weight, u_time, u_st, u_phase = read_upper_body('task_upper_body.txt')

for i in range(13):
    plt.figure()
    plt.plot(u_time, u_pos[:,i])
    plt.plot(u_time, u_pos_d[:, i])


plt.show()