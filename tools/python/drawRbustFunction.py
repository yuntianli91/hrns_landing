#coding=utf-8
###################### Python Module ##########################
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
##################### Custom Function #########################

########################### Main ##############################
sampleNum = 1000
x = np.linspace(1, 10, sampleNum)
y = np.random.normal(0., 1., sampleNum)
J = y * y * 0.5

mag = 10.;
y_corrupt = y;
y_corrupt[400:500] += mag * np.ones(100)
J_corrupt = y_corrupt * y_corrupt * 0.5
## ------------ huber -------------------- ##
J_huber = np.zeros(sampleNum)

rho = 1.345
for i in range(0, sampleNum):
    e = y_corrupt[i]
    if (e < 1.345):
        J_huber[i] = e * e * 0.5
    else:
        J_huber[i] = 1.345 * (np.linalg.norm(e) - e * 0.5)
## ================ cauchy ================= ##
J_cauchy = np.zeros(sampleNum)

c = 2.3849
for i in range(0, sampleNum):
    e = y_corrupt[i]
    J_cauchy[i] = c * c * 0.5 * math.log(1. + math.pow(e / c, 2))
## ================= G-M =================== ##
J_gm = np.zeros(sampleNum)

sigma = 2.0
for i in range(0, sampleNum):
    e = y_corrupt[i]
    J_gm[i] = c * c * 0.5 / (sigma + e * e)

########################### plot ##############################
fig1,ax = plt.subplots(1,1)
ax.plot(x, J, '-b')
ax.plot(x, J_corrupt, '-g')
ax.plot(x, J_huber, '--', color='orange')
ax.plot(x, J_cauchy, '--', color='tomato')
ax.plot(x, J_gm, '--', color='steelblue')
plt.show()