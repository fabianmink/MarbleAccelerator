# -*- coding: utf-8 -*-
"""
Created on Sat Apr 18 11:17:21 2026

@author: Fabian Mink
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata


values = np.genfromtxt("point_output.csv", delimiter=",")

#Get data
x = values[:,0]
y = values[:,1]
A = values[:,2]
Br = values[:,3]
Bz = values[:,4]

B = np.sqrt(Br**2 + Bz**2)

# Create grid
xi = np.linspace(x.min(), x.max(), 100)
yi = np.linspace(y.min(), y.max(), 100)
xi, yi = np.meshgrid(xi, yi)


# Interpolate
Ai = griddata((x, y), A, (xi, yi), method='cubic')  # 'linear' and 'nearest' are also options
Bi = griddata((x, y), B, (xi, yi), method='cubic')  # 'linear' and 'nearest' are also options
#Bri = griddata((x, y), Br, (xi, yi), method='cubic')  # 'linear' and 'nearest' are also options
#Bzi = griddata((x, y), Bz, (xi, yi), method='cubic')  # 'linear' and 'nearest' are also options

#  Plot

#cmap 
#cmap='jet'
#cmap='turbo'
#cmap='rainbow'
contourf = plt.contourf(xi, yi, Bi, levels=100, norm='linear', cmap='jet')
contour = plt.contour(xi, yi, Ai, levels=19, colors='k', linewidths=.5)
#plt.quiver(xi, yi, Bri, Bzi)

#plt.clabel(contour, inline=1, fontsize=10)


plt.xlabel('r')
plt.ylabel('z')
plt.show()