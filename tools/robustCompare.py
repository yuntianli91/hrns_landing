#coding=utf-8
#####################################################################
########################### Python Module ###########################
#####################################################################
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
import seaborn as sns
import sys, os
import pylab as mpl

# mpl.rcParams['font.sans-serif'] = ['STSong']
#####################################################################
######################### Custom Function ###########################
#####################################################################
def autolabel(ax, rects, scale):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for rect in rects:
        height = rect.get_height()
        ax.annotate('{}'.format(round(height/ scale, 2)),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom')
#####################################################################
########################### Main Function ###########################
#####################################################################

#####################################################################
########################### Plot Function ###########################
#####################################################################
colors = ['royalblue', 'tomato', 'limegreen', 'violet', 'gold']
# labels = ['filter1', 'filter2', 'filter3', 'filter4']
labels = ['Huber', 'Cauchy', 'SC-DCS', 'GM', 'L2']

# colors = ['gold', 'tomato']
# labels = ['L2', 'Cauchy', 'Improvement']
#################### fig1 ####################
# armes = np.zeros((3, 7))
# idx = [0.,0.05,0.10,0.15,0.20,0.25,0.30]
# armes[0,:] = [26.42,26.70,27.65,27.78,28.23,27.97,28.02]
# armes[1,:] = [26.58,25.97,26.76,26.27,26.49,25.81,25.79]
# armes[2,:] = [0.6,2.73,3.22,5.44,6.16,7.74,7.96]

armes = np.zeros((3, 5))
idx = [0.,0.05,0.10,0.15,0.20]
armes = pd.read_csv('./rmsePg.csv').values
# armes[0,:] = [26.42,26.70,27.65,27.78,28.23]
# armes[1,:] = [26.58,25.97,26.76,26.27,26.49]
# armes[2,:] = [0.6,2.73,3.22,5.44,6.16]



fig3, ax1 = plt.subplots(1,1,figsize=(6,4.5))
# ax2 = ax1.twinx()

ax1.plot(idx, armes[:,0], '--d', color=colors[0], label=labels[0])
ax1.plot(idx, armes[:,1], '--X', color=colors[1], label=labels[1])
ax1.plot(idx, armes[:,2], '--.', color=colors[2], label=labels[2])
ax1.plot(idx, armes[:,3], '--p', color=colors[3], label=labels[3])
ax1.plot(idx, armes[:,4], '--*', color=colors[4], label=labels[4])
ax1.set_ylabel('$P_{ARMSE}$(m)')
ax1.set_xlabel('$p_g$')
# ax1.set_xlim([0,0.2])
ax1.set_ylim([25.5,28.5])
ax1.grid()
ax1.legend()
fig3.savefig('pg.pdf',format='pdf')
#################### fig2 ####################
armes = np.zeros((3, 5))
idx = [5,15,25,35,45]
# armes[0,:] = [26.35,26.68,26.87,27.56,27.93]
# armes[1,:] = [26.08,26.50,26.52,25.72,25.54]
# armes[2,:] = [1.04,0.65,1.31,5.65,8.54]


fig3, ax1 = plt.subplots(1,1,figsize=(6,4.5))
ax2 = ax1.twinx()
armes = pd.read_csv('./rmseSigma.csv').values

ax1.plot(idx, armes[:,0], '--d', color=colors[0], label=labels[0])
ax1.plot(idx, armes[:,1], '--X', color=colors[1], label=labels[1])
ax1.plot(idx, armes[:,2], '--.', color=colors[2], label=labels[2])
ax1.plot(idx, armes[:,3], '--p', color=colors[3], label=labels[3])
ax1.plot(idx, armes[:,4], '--*', color=colors[4], label=labels[4])
ax1.set_ylabel('$P_{ARMSE}$(m)')
ax1.set_xlabel('$\sigma_s(m)$')
# ax1.set_xlim([0,25])
ax1.set_ylim([25,28.5])
ax1.grid()
ax1.legend()
fig3.savefig('sigma.pdf',format='pdf')
################# fig interaction ############
plt.show()