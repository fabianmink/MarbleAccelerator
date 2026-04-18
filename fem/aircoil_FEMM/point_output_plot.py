# -*- coding: utf-8 -*-
"""
Created on Sat Apr 18 11:17:21 2026

@author: Fabian Mink
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.interpolate import griddata

values = np.genfromtxt("point_output.csv", delimiter=",")

#Get data
x = values[:,0]
y = values[:,1]
A = values[:,2]
Br = values[:,3]
Bz = values[:,4]

B = np.sqrt(Br**2 + Bz**2)


# #INTERPOLATE
# # Create grid
# xiv = np.linspace(x.min(), x.max(), 50)
# yiv = np.linspace(y.min(), y.max(), 100)
# xi, yi = np.meshgrid(xiv, yiv)
# # Interpolate
# Ai = griddata((x, y), A, (xi, yi), method='cubic') 
# Bi = griddata((x, y), B, (xi, yi), method='cubic')

# "Reshape"; much faster although much more points
# Create grid
xiv = np.arange(0,30,0.1)
yiv = np.arange(-30,30,0.1)
xi, yi = np.meshgrid(xiv, yiv)
# Interpolate
Ai = griddata((x, y), A, (xi, yi), method='nearest')  
Bi = griddata((x, y), B, (xi, yi), method='nearest')


Aif = np.concatenate((np.flip(Ai[:,1:],1), Ai),1)
Bif = np.concatenate((np.flip(Bi[:,1:],1), Bi),1)
xivf = np.concatenate((-np.flip(xiv[1:]), xiv))


#  Plot
fig, ax = plt.subplots(1, 1)

#cmap 
#cmap='jet'
#cmap='turbo'
#cmap='rainbow'
#contourf = axs.contourf(xiv, yiv, Bi, levels=100, norm='linear', cmap='jet', vmin=0, vmax=0.2)
#contour = axs.contour(xiv, yiv, Ai, levels=19, colors='k', linewidths=.5)

contourf = ax.contourf(xivf, yiv, Bif, levels=200, norm='linear', cmap='jet', vmin=0, vmax=0.2)
#contour = ax.contour(xivf, yiv, Aif, levels=19, colors='k', linewidths=.5)
contour = ax.contour(xivf, yiv, Aif, np.linspace(Aif.min(),Aif.max(),20)[1:], colors='k', linewidths=.5) #exclude 0

ax.set_aspect('equal', 'box')


marble = patches.Circle((0, -15), 12.7/2, color='black', fill=False)
coilr =  patches.Rectangle((8.4, -7.5), 7.6, 15, color='black', fill=False)
coill =  patches.Rectangle((-8.4, -7.5), -7.6, 15, color='black', fill=False)


# Add the circle to the axes
ax.add_patch(marble)
ax.add_patch(coilr)
ax.add_patch(coill)

#plt.quiver(xi, yi, Bri, Bzi)
#plt.clabel(contour, inline=1, fontsize=10)

#ax.grid(1)

plt.xlabel(r'$x/\mathrm{mm}$')
plt.ylabel(r'$z/\mathrm{mm}$')
plt.show()