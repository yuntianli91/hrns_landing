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
calc = True
filename = "periodRmse.csv"
filterNum = 2
colNames = ['time']
for i in range(0, filterNum):
    colNames.append('P_F' + str(i+1))
for i in range(0, filterNum):
    colNames.append('V_F' + str(i+1))

if calc:
    datapath = "../output/diffPeriod/"
    trajData = pd.read_csv('../data/stdTraj/caGeo.csv')
    filterData = pd.read_csv(datapath + "AA0.csv");

    NN = filterData.shape[0]
    rmseAll = np.zeros([NN, 2 * filterNum + 1])

    start_idx = 0
    end_idx = 1
    for i in range(start_idx, end_idx):
        f1Data = pd.read_csv(datapath + "AA" + str(i) + ".csv")
        f2Data = pd.read_csv(datapath + "AR" + str(i) + ".csv")
        #calculate error
        f1Err = calcError(trajData, f1Data)
        f2Err = calcError(trajData, f2Data)
        # assign time
        if i == start_idx:
            rmseAll[:,0] = f1Err[:,0]
        # compute rmse
        f1Rmse = calcRMSE(f1Err, 3)
        f2Rmse = calcRMSE(f2Err, 3)
        # sum rmse
        rmseAll[:,1] += f1Rmse[:,1]
        rmseAll[:,2] += f2Rmse[:,1]
 
        rmseAll[:,3] += f1Rmse[:,2]
        rmseAll[:,4] += f2Rmse[:,2]

    rmseAll[:,1:] /= (end_idx - start_idx)

    rmseAllDf = pd.DataFrame(data=rmseAll, columns=colNames)
    rmseAllDf.to_csv(filename,index=None)
else:    
    rmseAllDf = pd.read_csv(filename)

#########################################################################
rmseStatistic = np.zeros([2, 2 * filterNum])
rmseStatistic[0,:] = rmseAllDf.mean()[1:,]

for i in range(0, filterNum):
    if(i > 0 and rmseStatistic[0,i-1] != 0 and rmseStatistic[0,i] != 0 ):
        rmseStatistic[1,i] = abs(rmseStatistic[0,i] - rmseStatistic[0,i-1]) / rmseStatistic[0,i-1] * 100
for i in range(0, 2):
    for j in range(0, filterNum):
        print('%7.3f '%rmseStatistic[i, j], end='')
    print()
#########################################################################
colors = ['royalblue', 'tomato']
labels = ['L2', 'Cauchy']
sep = 8
scaleSep = 8
############### position rmse ############### 
fig1, ax = plt.subplots(1, 1, figsize=(7,5))
ax.plot(rmseAllDf['time'].values[::scaleSep * sep], rmseAllDf['P_F1'].values[::scaleSep * sep], marker='d', color=colors[0])
ax.plot(rmseAllDf['time'].values[::scaleSep * sep], rmseAllDf['P_F2'].values[::scaleSep * sep], marker='X', color=colors[1])
############### velocity rmse ############### 
fig2, ax = plt.subplots(1, 1, figsize=(7,5))
ax.plot(rmseAllDf['time'].values[::scaleSep * sep], rmseAllDf['V_F1'].values[::scaleSep * sep], marker='d', color=colors[0])
ax.plot(rmseAllDf['time'].values[::scaleSep * sep], rmseAllDf['V_F2'].values[::scaleSep * sep], marker='X', color=colors[1])
############### interaction ############### 
plt.show()


