#coding=utf-8
######################## 导入模块 #######################
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import systems as sys
######################## 自定义函数 ######################
def quat2euler(quat):
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]
    yaw = math.atan(round(2.0 * (q1 * q2 - q0 * q3) / (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3), 6))
    pitch = math.asin(round(-2.0 * (q0 * q2 + q1 * q3), 6))
    roll = math.atan(2.0 * (q2 * q3 - q0 * q1) / (q0 * q0 + q3 * q3 - q2 * q2 - q1 * q1))
    return [roll, pitch, yaw]

def drawMoon(axis, radius, filled):
    # data
    center = [0., 0., 0.]
    u = np.linspace(-0.5 * np.pi, 0.5 * np.pi, 100)
    v = np.linspace(0, 0.5 * np.pi, 100)
    x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
    y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]

    if filled:
        axis.plot_surface(x, y, z, rstride=2, cstride=2, color='lightgrey', alpha=0.5)
    else:
        axis.plot_wireframe(x, y, z, rstride=5, cstride=5, color='silver')
###################### 读取标称轨迹数据（csv格式） ########################
traj_data = pd.read_csv('../data/standardTraj/trajMCMF.csv').values
imu_data = pd.read_csv('../data/sensorSimData/imuData.csv').values
# ned_data = pd.read_csv('../data/posNED.csv').values
# ned_data_imu = pd.read_csv('../data/posMCMF.csv').values
# beacon_location = pd.read_csv('/home/yuntian/dataset/simulator/lander/beacon_location.csv').values
###################### 提取各数据序列（注意python切片不包括尾部） ####################
time_series = traj_data[:,0]
pos = traj_data[:,1:4]
quat = traj_data[:,4:8] 
euler = traj_data[:,8:11]
vel = traj_data[:,11:14]
gyr = traj_data[:,14:17]
acc = traj_data[:,17:20]

time_imu = imu_data[:,0]
pos_imu = imu_data[:,1:4]
quat_imu = imu_data[:,4:8]
euler_imu = imu_data[:,8:11]
vel_imu = imu_data[:,11:14]
gyr_imu = imu_data[:,14:17]
acc_imu = imu_data[:,17:20]

# beacon_loc = beacon_location[:,4:7]
pos = pos / 1e3
pos_imu = pos_imu / 1e3

# ned_data = ned_data / 1000.
# ned_data_imu = ned_data_imu / 1000.

# N = len(ned_data[:,0])
# downRange = np.zeros(N)
# downRangeImu = np.zeros(N)
# for i in range(0, N):   
#     downRange[i] = math.sqrt(ned_data[i,0] * ned_data[i,0] + ned_data[i,1] * ned_data[i,1])
#     downRangeImu[i] = math.sqrt(ned_data_imu[i,0] * ned_data_imu[i,0] + ned_data_imu[i,1] * ned_data_imu[i,1])

######################### 画图 #########################
# 图注使用$$开启数学环境
###### figure1 #####
fig1, axes = plt.subplots(3, 1)
fig1.subplots_adjust(hspace=0.5)
## 子图1
axes[0].plot(time_series, acc[:,0], 'b-')
axes[0].plot(time_series, acc_imu[:,0], 'r-')
# axes[0].set_xlim(0, 220)
# axes[0].set_ylim(-3, 3)
axes[0].set_xlabel('$Time (s)$')
axes[0].set_ylabel('$a_x (m/s^2)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[0].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(1)
# axes[0].yaxis.set_major_locator(y_major_locator)
axes[0].grid()
## 子图2
axes[1].plot(time_series, acc[:,1], 'b-')
axes[1].plot(time_series, acc_imu[:,1], 'r-')
# axes[1].set_xlim(0, 220)
axes[1].set_ylim(-3, 3)
axes[1].set_xlabel('$Time (s)$')
axes[1].set_ylabel('$a_y (m/s^2)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[1].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(1)
# axes[1].yaxis.set_major_locator(y_major_locator)
axes[1].grid()
## 子图3
axes[2].plot(time_series, acc[:,2], 'b-')
axes[2].plot(time_series, acc_imu[:,2], 'r-')
# axes[2].set_xlim(0, 220)
axes[2].set_ylim(-3, 3)
axes[2].set_xlabel('$Time (s)$')
axes[2].set_ylabel('$a_z (m/s^2)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[2].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(1)
# axes[2].yaxis.set_major_locator(y_major_locator)
axes[2].grid()

fig1.savefig('acc.pdf', format='pdf')
# ##### figure2 #####
fig2, axes = plt.subplots(3, 1)
fig2.subplots_adjust(hspace=0.5)
## 子图1
axes[0].plot(time_series, gyr[:,0], 'b-')
axes[0].plot(time_series, gyr_imu[:,0], 'r-')
# axes[0].set_xlim(0, 220)
# axes[0].set_ylim(-0.075, 0.075)
axes[0].set_xlabel('$Time (s)$')
axes[0].set_ylabel('$\omega_x (rad/s)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[0].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[0].yaxis.set_major_locator(y_major_locator)
axes[0].grid()
## 子图2
axes[1].plot(time_series, gyr[:,1], 'b-')
axes[1].plot(time_series, gyr_imu[:,1], 'r-')
# axes[1].set_xlim(0, 220)
# axes[1].set_ylim(-0.075, 0.075)
axes[1].set_xlabel('$Time (s)$')
axes[1].set_ylabel('$\omega_y (rad/s)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[1].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[1].yaxis.set_major_locator(y_major_locator)
axes[1].grid()
## 子图3
axes[2].plot(time_series, gyr[:,2], 'b-')
axes[2].plot(time_series, gyr_imu[:,2], 'r-')
# axes[2].set_xlim(0, 220)
# axes[2].set_ylim(-0.075, 0.075)
axes[2].set_xlabel('$Time (s)$')
axes[2].set_ylabel('$\omega_z (rad/s)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[2].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[2].yaxis.set_major_locator(y_major_locator)
axes[2].grid()

fig2.savefig('gyr.pdf', format='pdf')

##### figure3 #####
fig3, axes = plt.subplots(3, 1)
fig3.subplots_adjust(hspace=0.5)

## 子图1
axes[0].plot(time_series, pos[:,0], 'b-')
axes[0].plot(time_series, pos_imu[:,0], 'r-')
# axes[0].set_xlim(0, 220)
# axes[0].set_ylim(-0.1, 0.1)
axes[0].set_xlabel('$time (s)$')
# axes[0].set_ylabel('$Latitude(deg)$')
axes[0].set_ylabel('$x(km)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[0].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[0].yaxis.set_major_locator(y_major_locator)
axes[0].grid()
## 子图2
axes[1].plot(time_series, pos[:,1], 'b-')
axes[1].plot(time_series, pos_imu[:,1], 'r-')
# axes[1].set_xlim(0, 220)
# axes[1].set_ylim(-0.1, 0.1)
axes[1].set_xlabel('$time (s)$')
# axes[1].set_ylabel('$Logitude (deg)$')
axes[1].set_ylabel('$y(km)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[1].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[1].yaxis.set_major_locator(y_major_locator)
axes[1].grid()
## 子图3
axes[2].plot(time_series, pos[:,2], 'b-')
axes[2].plot(time_series, pos_imu[:,2], 'r-')
# axes[2].set_xlim(0, 220)
# axes[2].set_ylim(-0.1, 0.1)
axes[2].set_xlabel('$time (s)$')
# axes[2].set_ylabel('$Altimeter (km)$')
axes[2].set_ylabel('$z(km)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[2].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[2].yaxis.set_major_locator(y_major_locator)
axes[2].grid()

fig3.savefig('pos.pdf', format='pdf')

##### figure4 #####
fig4, axes = plt.subplots(3, 1)
fig4.subplots_adjust(hspace=0.5)

## 子图1
axes[0].plot(time_series, vel[:,0], 'b-')
axes[0].plot(time_series, vel_imu[:,0], 'r-')
# axes[0].set_xlim(0, 220)
# axes[0].set_ylim(-0.1, 0.1)
axes[0].set_xlabel('$Time (s)$')
axes[0].set_ylabel('$v_x (m/s)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[0].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[0].yaxis.set_major_locator(y_major_locator)
axes[0].grid()
## 子图2
axes[1].plot(time_series, vel[:,1], 'b-')
axes[1].plot(time_series, vel_imu[:,1], 'r-')
# axes[1].set_xlim(0, 220)
# axes[1].set_ylim(-0.1, 0.1)
axes[1].set_xlabel('$Time (s)$')
axes[1].set_ylabel('$v_y (m/s)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[1].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[1].yaxis.set_major_locator(y_major_locator)
axes[1].grid()
## 子图3
axes[2].plot(time_series, vel[:,2], 'b-')
axes[2].plot(time_series, vel_imu[:,2], 'r-')
# axes[2].set_xlim(0, 220)
# axes[2].set_ylim(-0.1, 0.1)
axes[2].set_xlabel('$Time (s)$')
axes[2].set_ylabel('$v_z (m/s)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[2].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[2].yaxis.set_major_locator(y_major_locator)
axes[2].grid()

fig4.savefig('vel.pdf', format='pdf')
##### figure5 #####
# # rad -> deg
# N = len(quat[:,1])
# euler = np.zeros([N,3], float)
# for i in range(0, N):
#     tmp = quat2euler(quat[i,:])
#     euler[i, 0] = tmp[0] / math.pi * 180.
#     euler[i, 1] = tmp[1] / math.pi * 180.
#     euler[i, 2] = tmp[2] / math.pi * 180.

# N = len(quat_imu[:,1])
# euler_imu = np.zeros([N,3], float)
# for i in range(0, N):
#     tmp = quat2euler(quat_imu[i,:])
#     euler_imu[i, 0] = tmp[0] / math.pi * 180.
#     euler_imu[i, 1] = tmp[1] / math.pi * 180.
#     euler_imu[i, 2] = tmp[2] / math.pi * 180.

# euler = euler / math.pi * 180.
# euler_imu = euler_imu / math.pi * 180.

fig5, axes = plt.subplots(3, 1)
fig5.subplots_adjust(hspace=0.5)
## 子图1
axes[0].plot(time_series, euler[:,0], '-b')
axes[0].plot(time_imu, euler_imu[:,0], '-r')
# axes[0].set_xlim(0, 220)
# axes[0].set_ylim(-0.1, 0.1)
axes[0].set_xlabel('$time (s)$')
axes[0].set_ylabel('$\theta (\deg)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[0].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[0].yaxis.set_major_locator(y_major_locator)
axes[0].grid()
## 子图2
axes[1].plot(time_series, euler[:,1], 'b-')
axes[1].plot(time_series, euler_imu[:,1], 'r-')
# axes[1].set_xlim(0, 220)
# axes[1].set_ylim(-0.1, 0.1)
axes[1].set_xlabel('$time (s)$')
axes[1].set_ylabel('$\theta_y (\deg)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[1].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[1].yaxis.set_major_locator(y_major_locator)
axes[1].grid()
## 子图3
axes[2].plot(time_series, euler[:,2], 'b-')
axes[2].plot(time_series, euler_imu[:,2], 'r-')
# axes[2].set_xlim(0, 220)
# axes[2].set_ylim(-0.1, 0.1)
axes[2].set_xlabel('$time (s)$')
axes[2].set_ylabel('$\theta_z (\deg)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[2].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[2].yaxis.set_major_locator(y_major_locator)
axes[2].grid()

fig5.savefig('att.pdf', format='pdf')
# # ##### figure6 #####
# fig6 = plt.figure(figsize=(8,8))
# ax = fig6.add_subplot(111, projection='3d')

# drawMoon(ax, 1737, 1)
# ax.scatter(pos_imu[0,0], pos_imu[0,1], pos_imu[0,2], color='cyan', marker='*', lw=3)
# # ax.scatter(pos_imu[-1,0], pos_imu[-1,1], pos_imu[-1,2], color='orange', marker='D', lw=3)
# # ax.plot(pos[:,0], pos[:,1], pos[:,2], color='royalblue')
# ax.plot(pos_imu[:,0], pos_imu[:,1], pos_imu[:,2], color='tomato', lw=2)

# # ax.view_init(10,-85)

# # ax.set_xticks([])
# # ax.set_yticks([])
# # ax.set_zticks([])

# ax.set_xlabel("x(km)")
# ax.set_ylabel("y(km)")
# ax.set_zlabel("z(km)")

# ax.grid()


# # ax.set_zlim([0, 700])
# # axis.plot(ned_data[:,3], ned_data[:,1])
# # axis.plot(ned_data_imu[:,3], ned_data_imu[:,1])
# # axis.grid()
# # axis.set_xlabel("East(km)")
# # axis.set_ylabel("North(km)")

# # fig6.savefig('ne.pdf', format='pdf')
# ##### figure7 ######
# fig7, axes = plt.subplots(3,1)
# fig7.subplots_adjust(hspace=0.5)
# # lat = np.arctan(pos_imu[:,2] / np.sqrt(pos_imu[:,0] * pos_imu[:,0] + pos_imu[:,1] * pos_imu[:,1]))
# # lon = np.arctan(pos_imu[:,1] / pos_imu[:,0])
# # alt = np.linalg.norm(pos_imu, axis=1) - 1737.

# axes[0].plot(time_series, pos[:,0])
# axes[0].plot(time_series, ned_data_imu[:,1])

# axes[1].plot(time_series, pos[:,2])
# axes[1].plot(time_series, ned_data_imu[:,2])

# axes[2].plot(time_series, pos[:,1])
# axes[2].plot(time_series, ned_data_imu[:,3])
# # ###### figure 8 #######
fig8, axes = plt.subplots(3,1)
fig8.subplots_adjust(hspace=0.5)

axes[0].plot(time_series, pos[:,0] - pos_imu[:,0]);
# axes[0].plot(time_series, pos[:,0] - ned_data_imu[:,1]);

axes[1].plot(time_series, pos[:,1] - pos_imu[:,1]);
# axes[1].plot(time_series, pos[:,2] - ned_data_imu[:,2]);

axes[2].plot(time_series, pos[:,2] - pos_imu[:,2]);
# axes[2].plot(time_series, pos[:,1] - ned_data_imu[:,3]);


# ######### figure 9 #######
# fig9, axes = plt.subplots(4,1)
# axes[0].plot(time_series, quat[:,0])
# axes[0].plot(time_series, quat_imu[:,0])

# axes[1].plot(time_series, quat[:,1])
# axes[1].plot(time_series, quat_imu[:,1])

# axes[2].plot(time_series, quat[:,2])
# axes[2].plot(time_series, quat_imu[:,2])

# axes[3].plot(time_series, quat[:,3])
# axes[3].plot(time_series, quat_imu[:,3])


##显示绘图
plt.show()







