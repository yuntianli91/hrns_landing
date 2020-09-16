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
def getDPos(trajData, virnsData):
    N1 = len(trajData[:,0])
    N2 = len(virnsData[:,0])

    dp = np.zeros((N2,4))
    cnt = 0
    lastPos = trajData[0,1:4]
    for i in range(0, N1):
        if(np.abs(trajData[i,0] - virnsData[cnt,0]) < 1e-5):
            dp[cnt,0] = virnsData[cnt,0]
            dp[cnt,1:4] = trajData[i,1:4] - lastPos
            lastPos = trajData[i,1:4]
            cnt = cnt + 1
            if(cnt == N2):
                break

    return dp

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

#####################################################
colors=['tab:blue', 'tab:red']
labels=['dPos', 'virns']
fig1, axes = plt.subplots(3,1,figsize=(6,4))

axes[0].plot(dPos[:,0], dPos[:,1], color=colors[0], label=labels[0])
axes[0].plot(virnsData[:,0], virnsData[:,1], color=colors[1], label=labels[1])

axes[1].plot(dPos[:,0], dPos[:,2], color=colors[0], label=labels[0])
axes[1].plot(virnsData[:,0], virnsData[:,2], color=colors[1], label=labels[1])

axes[2].plot(dPos[:,0], dPos[:,3], color=colors[0], label=labels[0])
axes[2].plot(virnsData[:,0], virnsData[:,3], color=colors[1], label=labels[1])
#####################################################
plt.show()