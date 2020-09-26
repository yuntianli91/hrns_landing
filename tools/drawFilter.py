#coding=utf-8
######################## 导入模块 #######################
import numpy as np
import pandas as pd
import math
import matplotlib
import matplotlib.pyplot as plt
import systems as sys
from matplotlib import rcParams

# matplotlib.use("pgf")
# pgf_config = {
#     "font.family":'serif',
#     "font.size": 10,
#     "pgf.rcfonts": False,
#     "text.usetex": True,
#     "pgf.preamble": [
#         r"\usepackage{unicode-math}",
#         #r"\setmathfont{XITS Math}", 
#         # 这里注释掉了公式的XITS字体，可以自行修改
#         r"\setmainfont{Times New Roman}",
#         r"\usepackage{xeCJK}",
#         r"\xeCJKsetup{CJKmath=true}",
#         r"\setCJKmainfont{SimSun}",
#     ],
# }
# # rcParams.update(pgf_config)
R_M = 1.737e6
######################## 自定义函数 ############################
def calcError(trajdata, filterdata):
    pos_0 = trajdata.iloc[:, 1:4].values
    pos_1 = filterdata.iloc[:, 1:4].values

    vel_0 = trajdata.iloc[:, 11:14].values
    vel_1 = filterdata.iloc[:, 4:7].values
    
    sigmaP = filterdata.iloc[:, 7:10].values
    sigmaV = filterdata.iloc[:, 10:13].values

    time_0 = trajdata.iloc[:,0].values
    time_1 = filterdata.iloc[:,0].values

    N = len(time_1)
    idx = 0
    err = np.zeros([N, 13])
    while idx < N:
        for i in range(0, len(time_0)):
            if time_0[i] == time_1[idx]:
                err[idx, 0] = time_1[idx]
                # err[idx, 1:4] = np.fabs(pos_1[idx, :] - pos_0[i, :])
                # err[idx, 4:7] = np.fabs(vel_1[idx, :] - vel_0[i, :])
                err[idx, 1:4] = pos_1[idx, :] - pos_0[i, :]
                err[idx, 1] = err[idx, 1] * (R_M + pos_1[idx, 1])
                err[idx, 3] = err[idx, 3] * (R_M + pos_1[idx, 1]) * math.cos(pos_1[idx,0])

                err[idx, 4:7] = vel_1[idx, :] - vel_0[i, :]
 
                err[idx, 7] = sigmaP[idx, 0] * (R_M + pos_1[idx, 1])
                err[idx, 8] = sigmaP[idx, 1]
                err[idx, 9] = sigmaP[idx, 2] * (R_M + pos_1[idx, 1]) * math.cos(pos_1[idx,0])

                idx += 1
                if(idx >= N):
                    break
    
    err[:,10:13] = sigmaV

    return err  

def calcRMSE(err, dim):
    N = len(err[:,0])
    M = round(len(err[0,:]) / 6) + 1
    
    rmse = np.zeros((N, M))
    rmse[:,0] = err[:,0]
    for i in range(0, N):
        for j in range(1, M):
            start_idx = -2 + 3 * j
            end_idx = start_idx + dim
            rmse[i, j] = math.sqrt(np.dot(err[i, start_idx:end_idx], err[i, start_idx:end_idx]) / dim)

    return rmse  
#################### 读取数据（csv格式） ########################
trajData = pd.read_csv('../data/stdTraj/caGeo.csv')
imuData = pd.read_csv('../data/sensorSimData/imuData.csv')
filter1Data = pd.read_csv('../output/ckfAA.csv')
filter2Data = pd.read_csv('../output/ckfAR.csv')

errFilter1 = calcError(trajData, filter1Data)
errFilter2 = calcError(trajData, filter2Data)

###################################################
drawSigma = True
labels = ['SC-SPKF','3$\sigma$']
# labels = ['标称轨迹','IMU轨迹']
colors = ['tab:blue','tab:red']

fig1, axes = plt.subplots(3, 1, figsize=(7,5))
fig1.subplots_adjust(hspace=0.5)

axes[0].plot(errFilter1[:,0], abs(errFilter1[:,1]) , color=colors[0])
axes[0].plot(errFilter2[:,0], abs(errFilter2[:,1]) , color=colors[1])
axes[0].set_xlabel("time(s)")
axes[0].set_ylabel("$\delta p_x$(m)")
axes[0].grid()

axes[1].plot(errFilter1[:,0], abs(errFilter1[:,2]) , color=colors[0])
axes[1].plot(errFilter2[:,0], abs(errFilter2[:,2]) , color=colors[1])
axes[1].set_xlabel("time(s)")
axes[1].set_ylabel("$\delta p_y$(m)")
axes[1].grid()

axes[2].plot(errFilter1[:,0], abs(errFilter1[:,3]) , color=colors[0])
axes[2].plot(errFilter2[:,0], abs(errFilter2[:,3]) , color=colors[1])
axes[2].set_xlabel("time(s)")
axes[2].set_ylabel("$\delta p_z$(m)")
axes[2].grid()

if(drawSigma):
    axes[0].plot(errFilter1[:,0], errFilter1[:,7], '--k')
    axes[0].plot(errFilter1[:,0], -errFilter1[:,7], '--k')
    axes[1].plot(errFilter1[:,0], errFilter1[:,8], '--k')
    axes[1].plot(errFilter1[:,0], -errFilter1[:,8], '--k')
    axes[2].plot(errFilter1[:,0], errFilter1[:,9], '--k')
    axes[2].plot(errFilter1[:,0], -errFilter1[:,9], '--k')
    
axes[0].legend(labels, loc='upper right')
axes[1].legend(labels, loc='upper right')
axes[2].legend(labels, loc='upper right')
# ####################################################
fig2, axes = plt.subplots(3, 1, figsize=(7,5))
fig2.subplots_adjust(hspace=0.5)

axes[0].plot(errFilter1[:,0], abs(errFilter1[:,4]) , color=colors[0])
axes[0].plot(errFilter2[:,0], abs(errFilter2[:,4]) , color=colors[1])
axes[0].set_xlabel("time(s)")
axes[0].set_ylabel("$\delta v_x$(m/s)")
axes[0].grid()

axes[1].plot(errFilter1[:,0], abs(errFilter1[:,5]) , color=colors[0])
axes[1].plot(errFilter2[:,0], abs(errFilter2[:,5]) , color=colors[1])
axes[1].set_xlabel("time(s)")
axes[1].set_ylabel("$\delta v_y$(m/s)")
axes[1].grid()

axes[2].plot(errFilter1[:,0], abs(errFilter1[:,6]) , color=colors[0])
axes[2].plot(errFilter2[:,0], abs(errFilter2[:,6]) , color=colors[1])
axes[2].set_xlabel("time(s)")
axes[2].set_ylabel("$\delta v_z$(m/s)")
axes[2].grid()

if(drawSigma):
    axes[0].plot(errFilter1[:,0], errFilter1[:,10], '--k')
    axes[0].plot(errFilter1[:,0], -errFilter1[:,10], '--k')
    axes[1].plot(errFilter1[:,0], errFilter1[:,11], '--k')
    axes[1].plot(errFilter1[:,0], -errFilter1[:,11], '--k')
    axes[2].plot(errFilter1[:,0], errFilter1[:,12], '--k')
    axes[2].plot(errFilter1[:,0], -errFilter1[:,12], '--k')
    
axes[0].legend(labels, loc='upper right')
axes[1].legend(labels, loc='upper right')
axes[2].legend(labels, loc='upper right')
####################################################

plt.show()