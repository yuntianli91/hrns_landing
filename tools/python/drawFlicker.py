#coding=utf-8
import numpy as np
import matplotlib.pyplot as plt

gauss1 = np.random.normal(0., 1.0, 1000)
gauss2 = np.random.normal(0., 0.5, 1000)
alpha = 0.3

gauss3 = alpha * gauss1 + (1.0 - alpha) + gauss2


fig, ax = plt.subplots(2,1)
ax[0].plot(gauss1,ls='--', color='royalblue')
ax[0].plot(gauss2,ls='--', color='orange')
ax[1].plot(gauss3,ls='-', color='tomato')

plt.show()