from plot_utils import *
import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)

mass = 39.15342
zH = 0.69
lbound_time = 0  #5
ubound_time = 100  #8

# trAlip2 = all_trajectories('one_step_MPC/Alip2Swing_trajectory.txt')
# trRobotSwing = readRobotSwTr('one_step_MPC/robotSwingFootTraj.txt')
# CurrentComstate = readRobotSwTr('one_step_MPC/RobotCOM.txt')
# RobotCommand = readRobotSwTr('one_step_MPC/RobotCommand.txt')
############################################################
##MPC FREQ ENV
############################################################
MpcComState_mpc_plus_rl = readRobotSwTr(
    'freq_env_Ly_range_v3_2/MpcCOMstate.txt')
COMmpcoor_mpc_plus_rl = readRobotSwTr(
    'freq_env_Ly_range_v3_2/RobotCOMmpcOri.txt')

MpcComState = readRobotSwTr('freq_env_Ly_range_MPC_new/MpcCOMstate.txt')
COMmpcoor = readRobotSwTr('freq_env_Ly_range_MPC_new/RobotCOMmpcOri.txt')

############################################################
##One STEP ENV
############################################################
# MpcComState_mpc_plus_rl = readRobotSwTr('one_step_RL/MpcCOMstate.txt')
# COMmpcoor_mpc_plus_rl = readRobotSwTr('one_step_RL/RobotCOMmpcOri.txt')

# MpcComState = readRobotSwTr('one_step_MPC/MpcCOMstate.txt')
# COMmpcoor = readRobotSwTr('one_step_MPC/RobotCOMmpcOri.txt')

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
mpc_coor_L_x = COMmpcoor[:, 12]
mpc_coor_L_y = COMmpcoor[:, 13]
mpc_coor_L_z = COMmpcoor[:, 14]
alip_time = COMmpcoor[:, 15]
# torso_roll = COMmpcoor[:, 16]
# torso_pitch = COMmpcoor[:, 17]
# torso_yaw = COMmpcoor[:, 18]

time = alip_time
# timeCOM = alip_time

# swingXCommand = RobotCommand[:, 0]
# swingYCommand = RobotCommand[:, 1]
# swingZCommand = RobotCommand[:, 2]
# swingVxCommand = RobotCommand[:, 3]
# swingVyCommand = RobotCommand[:, 4]
# swingVzCommand = RobotCommand[:, 5]
# swingAxCommand = RobotCommand[:, 6]
# swingAyCommand = RobotCommand[:, 7]
# swingAzCommand = RobotCommand[:, 8]
# swingTimeCommand = RobotCommand[:, 9]
# end_foot_commandx = RobotCommand[:, 10]
# end_foot_commandy = RobotCommand[:, 11]
# end_foot_commandz = RobotCommand[:, 12]

MpcxCOM = MpcComState[:, 0]
MpcyCOM = MpcComState[:, 1]
MpczCOM = MpcComState[:, 2]
MpcLxCOM = MpcComState[:, 3]
MpcLyCOM = MpcComState[:, 4]
MpcLzCOM = MpcComState[:, 5]
MpctimeCOM = MpcComState[:, 6]
MpcLx_off = MpcComState[:, 7]
MpcLy_des = MpcComState[:, 8]
MpcMu = MpcComState[:, 9]
MpcLegWidth = MpcComState[:, 10]
MpczH = MpcComState[:, 11]
MpcMassCom = MpcComState[:, 12]
MpcTs = MpcComState[:, 13]

with open('freq_env_Ly_range_MPC_new/LandTime.txt', 'r') as file:
    landingTime = [float(line.strip()) for line in file]

inter = sliceTime(time, lbound_time, ubound_time)
landingTimes_in_range = [
    t for t in landingTime if lbound_time <= t <= ubound_time
]

light_grey = [0.85, 0.85, 0.85]
facecolors = [
    light_grey, 'grey', 'brown', 'red', 'orange', 'yellow', 'green', 'blue',
    'purple', 'crimson', 'white'
]
colors = [facecolors[0], facecolors[-1]]

mpc_coor_x_mpc_rl = COMmpcoor_mpc_plus_rl[:, 0]
mpc_coor_y_mpc_rl = COMmpcoor_mpc_plus_rl[:, 1]
mpc_coor_z_mpc_rl = COMmpcoor_mpc_plus_rl[:, 2]
mpc_coor_vx_mpc_rl = COMmpcoor_mpc_plus_rl[:, 3]
mpc_coor_vy_mpc_rl = COMmpcoor_mpc_plus_rl[:, 4]
mpc_coor_vz_mpc_rl = COMmpcoor_mpc_plus_rl[:, 5]
mpc_coor_Lst_x_mpc_rl = COMmpcoor_mpc_plus_rl[:, 6]
mpc_coor_Lst_y_mpc_rl = COMmpcoor_mpc_plus_rl[:, 7]
mpc_coor_Lst_z_mpc_rl = COMmpcoor_mpc_plus_rl[:, 8]
mpc_coor_Lc_x_mpc_rl = COMmpcoor_mpc_plus_rl[:, 9]
mpc_coor_Lc_y_mpc_rl = COMmpcoor_mpc_plus_rl[:, 10]
mpc_coor_Lc_z_mpc_rl = COMmpcoor_mpc_plus_rl[:, 11]
mpc_coor_L_x_mpc_rl = COMmpcoor_mpc_plus_rl[:, 12]
mpc_coor_L_y_mpc_rl = COMmpcoor_mpc_plus_rl[:, 13]
mpc_coor_L_z_mpc_rl = COMmpcoor_mpc_plus_rl[:, 14]
alip_time_mpc_rl = COMmpcoor_mpc_plus_rl[:, 15]
time_mpc_rl = alip_time_mpc_rl

MpcxCOM_mpc_rl = MpcComState_mpc_plus_rl[:, 0]
MpcyCOM_mpc_rl = MpcComState_mpc_plus_rl[:, 1]
MpczCOM_mpc_rl = MpcComState_mpc_plus_rl[:, 2]
MpcLxCOM_mpc_rl = MpcComState_mpc_plus_rl[:, 3]
MpcLyCOM_mpc_rl = MpcComState_mpc_plus_rl[:, 4]
MpcLzCOM_mpc_rl = MpcComState_mpc_plus_rl[:, 5]
MpctimeCOM_mpc_rl = MpcComState_mpc_plus_rl[:, 6]
MpcLx_off_mpc_rl = MpcComState_mpc_plus_rl[:, 7]
MpcLy_des_mpc_rl = MpcComState_mpc_plus_rl[:, 8]
MpcMu_mpc_rl = MpcComState_mpc_plus_rl[:, 9]
MpcLegWidth_mpc_rl = MpcComState_mpc_plus_rl[:, 10]
MpczH_mpc_rl = MpcComState_mpc_plus_rl[:, 11]
MpcMassCom_mpc_rl = MpcComState_mpc_plus_rl[:, 12]
MpcTs_mpc_rl = MpcComState_mpc_plus_rl[:, 13]

with open('freq_env_Ly_range_v3_2/LandTime.txt', 'r') as file:
    landingTime = [float(line.strip()) for line in file]

inter_mpc_rl = sliceTime(time_mpc_rl, lbound_time, ubound_time)
