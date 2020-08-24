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
trajData = pd.read_csv("../data/standardTraj/trajMCMF.csv").values
cnsData = pd.read_csv("../data/sensorSimData/cnsData.csv").values
virnsData = pd.read_csv("../data/sensorSimData/virnsData.csv").values
cmnsData = pd.read_csv("../data/sensorSimData/cmnsData.csv").values

dPos = getDPos(trajData, virnsData)
errPos = getError(trajData, cmnsData)

######################################################
######################################################
fig1, axes = plt.subplots(3,1)
fig1.subplots_adjust(hspace=0.5)

axes[0].plot(cnsData[:,0], cnsData[:,5], color='tab:orange')
axes[0].plot(trajData[:,0], trajData[:,8], color='tab:blue')

axes[1].plot(cnsData[:,0], cnsData[:,6], color='tab:orange')
axes[1].plot(trajData[:,0], trajData[:,9], color='tab:blue')

axes[2].plot(cnsData[:,0], cnsData[:,7], color='tab:orange')
axes[2].plot(trajData[:,0], trajData[:,10], color='tab:blue')
######################################
fig2, axes = plt.subplots(3,2, figsize=(8,6))
fig2.subplots_adjust(hspace=0.5)

axes[0][0].plot(virnsData[:,0], virnsData[:,1], color='tab:orange')
axes[0][0].plot(dPos[:,0], dPos[:,1], color='tab:blue')
axes[0][0].set_xlabel("tims(s)")
axes[0][0].set_ylabel("$delta P_x$(m)")
axes[0][0].grid()

axes[1][0].plot(virnsData[:,0], virnsData[:,2], color='tab:orange')
axes[1][0].plot(dPos[:,0], dPos[:,2], color='tab:blue')
axes[1][0].set_xlabel("tims(s)")
axes[1][0].set_ylabel("$delta P_y$(m)")
axes[1][0].grid()

axes[2][0].plot(virnsData[:,0], virnsData[:,3], color='tab:orange')
axes[2][0].plot(dPos[:,0], dPos[:,3], color='tab:blue')
axes[2][0].set_xlabel("tims(s)")
axes[2][0].set_ylabel("$delta P_z$(m)")
axes[2][0].grid()

axes[0][1].plot(virnsData[:,0], virnsData[:,4], color='tab:orange')
axes[0][1].plot(trajData[:,0], trajData[:,1], color='tab:blue')
axes[0][1].set_xlabel("tims(s)")
axes[0][1].set_ylabel("$P_x$(m)")
axes[0][1].grid()

axes[1][1].plot(virnsData[:,0], virnsData[:,5], color='tab:orange')
axes[1][1].plot(trajData[:,0], trajData[:,2], color='tab:blue')
axes[1][1].set_xlabel("tims(s)")
axes[1][1].set_ylabel("$P_y$(m)")
axes[1][1].grid()

axes[2][1].plot(virnsData[:,0], virnsData[:,6], color='tab:orange')
axes[2][1].plot(trajData[:,0], trajData[:,3], color='tab:blue')
axes[2][1].set_xlabel("tims(s)")
axes[2][1].set_ylabel("$P_z$(m)")
axes[2][1].grid()

######################################
fig3, axes = plt.subplots(3,2, figsize=(8,6))
fig3.subplots_adjust(hspace=0.5)

axes[0][0].plot(trajData[:,0], trajData[:,1])
axes[0][0].plot(cmnsData[:,0], cmnsData[:,1])

axes[1][0].plot(trajData[:,0], trajData[:,2])
axes[1][0].plot(cmnsData[:,0], cmnsData[:,2])

axes[2][0].plot(trajData[:,0], trajData[:,3])
axes[2][0].plot(cmnsData[:,0], cmnsData[:,3])

axes[0][1].plot(errPos[:,0], errPos[:,1], color='tab:orange')
axes[1][1].plot(errPos[:,0], errPos[:,2], color='tab:orange')
axes[2][1].plot(errPos[:,0], errPos[:,3], color='tab:orange')

fig2.savefig('virns.pdf', format='pdf')
#####################################################
#####################################################
plt.show()