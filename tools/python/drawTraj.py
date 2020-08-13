#coding=utf-8
######################## 导入模块 #######################
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
import systems as sys
######################## 自定义函数 ######################
def quat2euler(quat):
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]
    yaw = math.atan(2.0 * (q1 * q2 - q0 * q3) / (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3))
    pitch = math.asin(-2.0 * (q0 * q2 + q1 * q3))
    roll = math.atan(2.0 * (q2 * q3 - q0 * q1) / (q0 * q0 + q3 * q3 - q2 * q2 - q1 * q1))
    return [roll, pitch, yaw]
###################### 读取标称轨迹数据（csv格式） ########################
traj_data = pd.read_csv('../data/caGeo.csv').values
imu_data = pd.read_csv('../data/caGeo.csv').values
pos_data = pd.read_csv('../data/posNED.csv').values
# beacon_location = pd.read_csv('/home/yuntian/dataset/simulator/lander/beacon_location.csv').values
###################### 提取各数据序列（注意python切片不包括尾部） ####################
time_series = traj_data[:,0]
pos = traj_data[:,1:4]
quat = traj_data[:,4:8]
vel = traj_data[:,8:11]
gyr = traj_data[:,11:14]
acc = traj_data[:, 14:17]

time_imu = imu_data[:,0]
pos_imu = imu_data[:,1:4]
quat_imu = imu_data[:,4:8]
vel_imu = imu_data[:,8:11]
gyr_imu = imu_data[:,11:14]
acc_imu = imu_data[:, 14:17]

# beacon_loc = beacon_location[:,4:7]
######################### 画图 #########################
## 图注使用$$开启数学环境
# region
###### figure1 #####
fig1, axes = plt.subplots(3, 1)
fig1.subplots_adjust(hspace=0.5)
## 子图1
axes[0].plot(time_series, acc[:,0], 'b-')
# axes[0].set_xlim(0, 220)
axes[0].set_ylim(-3, 3)
axes[0].set_xlabel('$Time (s)$')
axes[0].set_ylabel('$a_x (m/s^2)$')

x_major_locator = plt.MultipleLocator(25)
axes[0].xaxis.set_major_locator(x_major_locator)
y_major_locator = plt.MultipleLocator(1)
axes[0].yaxis.set_major_locator(y_major_locator)
axes[0].grid()
## 子图2
axes[1].plot(time_series, acc[:,1], 'b-')
# axes[1].set_xlim(0, 220)
axes[1].set_ylim(-3, 3)
axes[1].set_xlabel('$Time (s)$')
axes[1].set_ylabel('$a_y (m/s^2)$')

x_major_locator = plt.MultipleLocator(25)
axes[1].xaxis.set_major_locator(x_major_locator)
y_major_locator = plt.MultipleLocator(1)
axes[1].yaxis.set_major_locator(y_major_locator)
axes[1].grid()
## 子图3
axes[2].plot(time_series, acc[:,2], 'b-')
# axes[2].set_xlim(0, 220)
axes[2].set_ylim(-3, 3)
axes[2].set_xlabel('$Time (s)$')
axes[2].set_ylabel('$a_z (m/s^2)$')

x_major_locator = plt.MultipleLocator(25)
axes[2].xaxis.set_major_locator(x_major_locator)
y_major_locator = plt.MultipleLocator(1)
axes[2].yaxis.set_major_locator(y_major_locator)
axes[2].grid()

fig1.savefig('acc.pdf', format='pdf')
# ##### figure2 #####
fig2, axes = plt.subplots(3, 1)
fig2.subplots_adjust(hspace=0.5)
## 子图1
axes[0].plot(time_series, gyr[:,0], 'b-')
# axes[0].set_xlim(0, 220)
# axes[0].set_ylim(-0.075, 0.075)
axes[0].set_xlabel('$Time (s)$')
axes[0].set_ylabel('$\omega_x (rad/s)$')

x_major_locator = plt.MultipleLocator(25)
axes[0].xaxis.set_major_locator(x_major_locator)
y_major_locator = plt.MultipleLocator(0.05)
axes[0].yaxis.set_major_locator(y_major_locator)
axes[0].grid()
## 子图2
axes[1].plot(time_series, gyr[:,1], 'b-')
# axes[1].set_xlim(0, 220)
# axes[1].set_ylim(-0.075, 0.075)
axes[1].set_xlabel('$Time (s)$')
axes[1].set_ylabel('$\omega_y (rad/s)$')

x_major_locator = plt.MultipleLocator(25)
axes[1].xaxis.set_major_locator(x_major_locator)
y_major_locator = plt.MultipleLocator(0.05)
axes[1].yaxis.set_major_locator(y_major_locator)
axes[1].grid()
## 子图3
axes[2].plot(time_series, gyr[:,2], 'b-')
# axes[2].set_xlim(0, 220)
# axes[2].set_ylim(-0.075, 0.075)
axes[2].set_xlabel('$Time (s)$')
axes[2].set_ylabel('$\omega_z (rad/s)$')

x_major_locator = plt.MultipleLocator(25)
axes[2].xaxis.set_major_locator(x_major_locator)
y_major_locator = plt.MultipleLocator(0.05)
axes[2].yaxis.set_major_locator(y_major_locator)
axes[2].grid()

fig2.savefig('gyr.pdf', format='pdf')

##### figure3 #####
fig3, axes = plt.subplots(3, 1)
pos[:,0:2]  = pos[:,0:2] / math.pi * 180.;
pos[:,2] = pos[:,2] / 1000.;
## 子图1
axes[0].plot(time_series, pos[:,0], 'b-')
# axes[0].set_xlim(0, 220)
# axes[0].set_ylim(-0.1, 0.1)
axes[0].set_xlabel('$time (s)$')
axes[0].set_ylabel('$Latitude(deg)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[0].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[0].yaxis.set_major_locator(y_major_locator)
axes[0].grid()
## 子图2
axes[1].plot(time_series, pos[:,1], 'b-')
# axes[1].set_xlim(0, 220)
# axes[1].set_ylim(-0.1, 0.1)
axes[1].set_xlabel('$time (s)$')
axes[1].set_ylabel('$Logitude (deg)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[1].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[1].yaxis.set_major_locator(y_major_locator)
axes[1].grid()
## 子图3
axes[2].plot(time_series, pos[:,2], 'b-')
# axes[2].set_xlim(0, 220)
# axes[2].set_ylim(-0.1, 0.1)
axes[2].set_xlabel('$time (s)$')
axes[2].set_ylabel('$Altimeter (km)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[2].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[2].yaxis.set_major_locator(y_major_locator)
axes[2].grid()

fig3.savefig('pos.pdf', format='pdf')

##### figure4 #####
fig4, axes = plt.subplots(3, 1)
## 子图1
axes[0].plot(time_series, vel[:,0], 'b-')
# axes[0].set_xlim(0, 220)
# axes[0].set_ylim(-0.1, 0.1)
axes[0].set_xlabel('$Time (s)$')
axes[0].set_ylabel('$v_N (m/s)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[0].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[0].yaxis.set_major_locator(y_major_locator)
axes[0].grid()
## 子图2
axes[1].plot(time_series, vel[:,1], 'b-')
# axes[1].set_xlim(0, 220)
# axes[1].set_ylim(-0.1, 0.1)
axes[1].set_xlabel('$Time (s)$')
axes[1].set_ylabel('$v_E (m/s)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[1].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[1].yaxis.set_major_locator(y_major_locator)
axes[1].grid()
## 子图3
axes[2].plot(time_series, vel[:,2], 'b-')
# axes[2].set_xlim(0, 220)
# axes[2].set_ylim(-0.1, 0.1)
axes[2].set_xlabel('$Time (s)$')
axes[2].set_ylabel('$v_D (m/s)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[2].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[2].yaxis.set_major_locator(y_major_locator)
axes[2].grid()

fig4.savefig('vel.pdf', format='pdf')
##### figure5 #####
## quat -> euler
N = len(quat[:,1])
euler = np.zeros([N,3], float)
for i in range(0, N):
    tmp = quat2euler(quat[i,:])
    euler[i, 0] = tmp[0] / math.pi * 180.
    euler[i, 1] = tmp[1] / math.pi * 180.
    euler[i, 2] = tmp[2] / math.pi * 180.

N = len(quat_imu[:,1])
euler_imu = np.zeros([N,3], float)
for i in range(0, N):
    tmp = quat2euler(quat_imu[i,:])
    euler_imu[i, 0] = tmp[0] / math.pi * 180.
    euler_imu[i, 1] = tmp[1] / math.pi * 180.
    euler_imu[i, 2] = tmp[2] / math.pi * 180.

fig5, axes = plt.subplots(3, 1)
## 子图1
axes[0].plot(time_series, euler[:,0], 'b-')
axes[0].plot(time_imu, euler_imu[:,0], '-r')
axes[0].set_xlim(0, 220)
# axes[0].set_ylim(-0.1, 0.1)
axes[0].set_xlabel('$time (s)$')
axes[0].set_ylabel('$Roll (\deg)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[0].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[0].yaxis.set_major_locator(y_major_locator)
axes[0].grid()
## 子图2
axes[1].plot(time_series, -euler[:,1], 'b-')
axes[1].plot(time_series, -euler_imu[:,1], 'r-')
axes[1].set_xlim(0, 220)
# axes[1].set_ylim(-0.1, 0.1)
axes[1].set_xlabel('$time (s)$')
axes[1].set_ylabel('$Pitch (\deg)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[1].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[1].yaxis.set_major_locator(y_major_locator)
axes[1].grid()
## 子图3
axes[2].plot(time_series, euler[:,2], 'b-')
axes[2].plot(time_series, euler_imu[:,2], 'r-')
axes[2].set_xlim(0, 220)
# axes[2].set_ylim(-0.1, 0.1)
axes[2].set_xlabel('$time (s)$')
axes[2].set_ylabel('$Yaw (\deg)$')

# x_major_locator = plt.MultipleLocator(25)
# axes[2].xaxis.set_major_locator(x_major_locator)
# y_major_locator = plt.MultipleLocator(0.05)
# axes[2].yaxis.set_major_locator(y_major_locator)
axes[2].grid()

fig5.savefig('att.pdf', format='pdf')
##### figure6 #####
fig6, axes = plt.subplots(1,2)
pos_data = pos_data / 1000.

axes[0].plot(pos_data[:,2], pos_data[:,1]);
axes[0].set_xlabel("East(km)")
axes[0].set_ylabel("North(km)")
# plt.axis('equal')
##显示绘图
plt.show()







