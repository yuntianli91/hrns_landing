#coding=utf-8
###################### Python Module ##########################
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
##################### Custom Function #########################

########################### Main ##############################
sampleNum = 1000
x = np.linspace(-5, 5, sampleNum)
## ================ L2 ===================== ##
rhoL2 = np.zeros((sampleNum, 2))
for i in range(0, sampleNum):
    rhoL2[i,0] = 0.5 * x[i] * x[i]
    rhoL2[i,1] = 1
## ================ Huber ================== ##
rhoHuber = np.zeros((sampleNum, 2))
k = 1
for i in range(0, sampleNum):
    if(abs(x[i]) <= k):
        rhoHuber[i,0] = 0.5 * x[i] * x[i]
        rhoHuber[i,1] = 1
    else:
        rhoHuber[i,0] = k * (abs(x[i]) - 0.5 * k);
        rhoHuber[i,1] = k / abs(x[i])
## ================ Cauchy ================= ##
rhoCauchy = np.zeros((sampleNum, 2))
k = 1
for i in range(0, sampleNum):
    rhoCauchy[i,0] = 0.5 * k * k * math.log(1. +  x[i] * x[i] / k)
    rhoCauchy[i,1] = 1 / (1 + x[i] * x[i] / (k * k))
## ================= G-M =================== ##
rhoGM = np.zeros((sampleNum, 2))
k = 1
for i in range(0, sampleNum):
    scale = (k + x[i] * x[i])
    rhoGM[i,0] = 0.5 * x[i] * x[i] / scale
    rhoGM[i,1] = k * k / (scale * scale) 
## ================= DCS =================== ##
rhoDCS = np.zeros((sampleNum,2))
k=1
for i in range(0, sampleNum):
    x2 = x[i] * x[i]
    if(x2 <= k):
        rhoDCS[i,0] = 0.5 * x2;
        rhoDCS[i,1] = 1
    else:
        scale = (k + x2)
        rhoDCS[i,0] = 2 * k * x2 / scale - 0.5 * k
        rhoDCS[i,1] = 4 * k * k / (scale * scale)
## ================== MCC =================== ##
rhoMCC = np.zeros((sampleNum, 2))
k = 1
for i in range(0, sampleNum):
    rhoMCC[i,0] = - k * k * math.exp(-x[i] * x[i]/ (2 * k * k))
    rhoMCC[i,1] = math.exp(-x[i] * x[i]/ (2 * k * k))
## ================== weight line ==================== ##
hk = np.linspace(-0.05, 1.05, 100)
kk = np.ones(100) * k
########################### plot ##############################
fig1, axes = plt.subplots(1,2,figsize=(10,4))
colors=['gold','royalblue', 'limegreen', 'violet', 'red', 'orange']
labels=['L2','Huber','Cauchy','GM', 'DCS', 'MCC']

for i in range(0, 2):
    axes[i].plot(x, rhoL2[:,i], color=colors[0])
    axes[i].plot(x, rhoHuber[:,i], color=colors[1])
    axes[i].plot(x, rhoCauchy[:,i], color=colors[2])
    axes[i].plot(x, rhoGM[:,i], color=colors[3])
    axes[i].plot(x, rhoDCS[:,i], color=colors[4])
    axes[i].plot(x, rhoMCC[:,i], color=colors[5])
    axes[i].grid()
    axes[i].set_xlabel('Error')

axes[1].plot(kk,hk,'--k')
axes[1].plot(-kk,hk,'--k')

axes[0].legend(labels,loc=0)
axes[1].legend(labels,loc=(0.7,0.5))
axes[0].set_ylabel('Cost')
axes[1].set_ylabel('Weight($k=1$)')
axes[1].set_ylim([-0.05,1.05])

fig1.savefig('kernel_function.pdf', format='pdf')


plt.show()