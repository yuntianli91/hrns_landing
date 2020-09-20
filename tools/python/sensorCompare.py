#coding=utf-8
######################## 导入模块 #######################
#######################################################
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import systems as sy
#######################################################
def geo2mcmf(geo):
    R_m = 1.737e6
    mcmf = np.array([0.,0.,0.])
    mcmf[0] = (R_m + geo[1]) * math.cos(geo[0]) * math.cos(geo[2])
    mcmf[1] = (R_m + geo[1]) * math.cos(geo[0]) * math.sin(geo[2])
    mcmf[2] = (R_m + geo[1]) * math.sin(geo[0])
    return mcmf

def getDPos(trajData, virnsData):
    N1 = len(trajData[:,0])
    N2 = len(virnsData[:,0])

    dp = np.zeros((N2,4))
    cnt = 0
    lastPos = geo2mcmf(trajData[0,1:4])
    for i in range(0, N1):
        if(np.abs(trajData[i,0] - virnsData[cnt,0]) < 1e-5):
            curPos = geo2mcmf(trajData[i,1:4])
            dp[cnt,0] = virnsData[cnt,0]
            dp[cnt,1:4] = curPos - lastPos
            lastPos = curPos
            cnt = cnt + 1
            if(cnt == N2):
                break
    return dp

def getPos(trajData, virnsData):
    N1 = len(trajData[:,0])
    N2 = len(virnsData[:,0])

    Pos = np.zeros((N2,4))
    cnt = 0
    for i in range(0, N1):
        if(np.abs(trajData[i,0] - virnsData[cnt,0]) < 1e-5):
            Pos[cnt,0] = virnsData[cnt,0]
            Pos[cnt,1:4] =geo2mcmf(trajData[i,1:4])
            cnt = cnt + 1
            if(cnt == N2):
                break 
    return Pos


def getError(trajData, cmnsData):
    N1 = len(trajData[:,0])
    N2 = len(cmnsData[:,0])

    errPos = np.zeros((N2,4))
    cnt = 0
    for i in range(0, N1):
        if(np.abs(trajData[i,0] - cmnsData[cnt,0]) < 1e-5):
            errPos[cnt,0] = cmnsData[cnt,0]
            errPos[cnt,1:4] = cmnsData[cnt,1:4] - trajData[i,1:4]
            cnt = cnt + 1
            if(cnt == N2):
                break 
    return errPos

#######################################################
trajData = pd.read_csv("../data/stdTraj/caGeo.csv").values
imuData = pd.read_csv("../data/sensorSimData/imuData.csv").values
virnsData = pd.read_csv("../data/sensorSimData/virnsData.csv").values
# cmnsData = pd.read_csv("../data/sensorSimData/cmnsData.csv").values
# cnsData = pd.read_csv("../data/sensorSimData/cnsData.csv").values

dPos = getDPos(trajData, virnsData)
trajPos = getPos(trajData, virnsData)

#####################################################
colors=['tab:blue', 'tab:red']
labels=['$\Delta p$', 'virns']
fig1, axes = plt.subplots(3,1,figsize=(8,6))
fig1.subplots_adjust(hspace=0.7)

axes[0].plot(dPos[1:,0], virnsData[1:,1], color=colors[1], label=labels[1])
axes[0].plot(dPos[1:,0], dPos[1:,1], color=colors[0], label=labels[0])
# axes[0].plot(dPos[1:,0], dPos[1:,1] - virnsData[1:,1], color=colors[0], label=labels[0])
axes[0].legend(loc='upper right',fontsize='small')
axes[0].grid()
axes[0].set_xlabel('t(s)')
axes[0].set_ylabel('$\Delta p_x$(m)')

axes[1].plot(dPos[1:,0], virnsData[1:,2], color=colors[1], label=labels[1])
axes[1].plot(dPos[1:,0], dPos[1:,2], color=colors[0], label=labels[0])
# axes[1].plot(dPos[1:,0], dPos[1:,2] - virnsData[1:,2], color=colors[0], label=labels[0])
axes[1].legend(loc='upper right',fontsize='small')
axes[1].grid()
axes[1].set_xlabel('t(s)')
axes[1].set_ylabel('$\Delta p_y$(m)')

axes[2].plot(dPos[1:,0], virnsData[1:,3], color=colors[1], label=labels[1])
axes[2].plot(dPos[1:,0], dPos[1:,3], color=colors[0], label=labels[0])
# axes[2].plot(dPos[1:,0], dPos[1:,3] - virnsData[1:,3], color=colors[0], label=labels[0])
axes[2].legend(loc='upper right',fontsize='small')
axes[2].grid()
axes[2].set_xlabel('t(s)')
axes[2].set_ylabel('$\Delta p_z$(m)')

fig1.savefig('virns_delta.pdf')
#####################################################
colors=['tab:blue', 'tab:red']
labels=['$p$', 'virns']
fig2, axes = plt.subplots(3,1,figsize=(8,6))
fig2.subplots_adjust(hspace=0.7)

axes[0].plot(trajPos[1:,0], trajPos[1:,1] / 1000, color=colors[0], label=labels[0])
axes[0].plot(virnsData[1:,0], virnsData[1:,4] / 1000, color=colors[1], label=labels[1])
axes[0].legend(loc='upper right',fontsize='small')
axes[0].grid()
axes[0].set_xlabel('t(s)')
axes[0].set_ylabel('$p_x$(km)')

axes[1].plot(trajPos[1:,0], trajPos[1:,2] / 1000, color=colors[0], label=labels[0])
axes[1].plot(virnsData[1:,0], virnsData[1:,5] / 1000, color=colors[1], label=labels[1])
axes[1].legend(loc='upper right',fontsize='small')
axes[1].grid()
axes[1].set_xlabel('t(s)')
axes[1].set_ylabel('$p_y$(km)')

axes[2].plot(trajPos[1:,0], trajPos[1:,3] / 1000, color=colors[0], label=labels[0])
axes[2].plot(virnsData[1:,0], virnsData[1:,6] / 1000, color=colors[1], label=labels[1])
axes[2].legend(loc='upper right',fontsize='small')
axes[2].grid()
axes[2].set_xlabel('t(s)')
axes[2].set_ylabel('$p_z$(km)')

fig2.savefig('virns.pdf')
#####################################################
plt.show()