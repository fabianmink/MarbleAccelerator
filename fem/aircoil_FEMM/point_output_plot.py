# -*- coding: utf-8 -*-
"""
Created on Sat Apr 18 11:17:21 2026

@author: Fabian Mink
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.interpolate import griddata


fem_data = np.genfromtxt('output.csv', delimiter=',')
z_file = fem_data[:,1];
Fz_file = fem_data[:,3];


for zm in range(-20,20,1):
#for zm in range(-20,-18,2):
    #values = np.genfromtxt("point_output.csv", delimiter=",")
    #values = np.genfromtxt("point_output_-15mm_20A.csv", delimiter=",")
    values = np.genfromtxt("point_output_" + str(zm) + "mm_20A.csv", delimiter=",")
    
    
    #Fz = 2.5
    Fz = np.interp(zm, z_file, Fz_file, left=None, right=None, period=None)
    print(Fz)
    
    
    #Get data
    x = values[:,0]
    y = values[:,1]
    A = values[:,2]
    Br = values[:,3]
    Bz = values[:,4]
    
    B = np.sqrt(Br**2 + Bz**2)
    
    #zm = -15
    
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
    xiv = np.arange(0,30,0.5)
    yiv = np.arange(-30,30,0.5)
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
        
    marble = patches.Circle((0, zm), 12.7/2, color='grey', fill=False, alpha=1)
    coilr =  patches.Rectangle((8.4, -7.5), 7.6, 15, color='brown', fill=False, alpha=1)
    coill =  patches.Rectangle((-8.4, -7.5), -7.6, 15, color='brown', fill=False, alpha=1)
    
    contourf = ax.contourf(xivf, yiv, Bif, levels=200, norm='linear', cmap='jet', vmin=0, vmax=0.5)
    #contour = ax.contour(xivf, yiv, Aif, levels=19, colors='k', linewidths=.5)
    contour = ax.contour(xivf, yiv, Aif, np.linspace(Aif.min(),Aif.max(),20)[1:], colors='k', linewidths=.5) #exclude 0
    
    #ax.arrow(0,zm,0,Fz*5,width=0.01,head_width=1.5,head_length=2,length_includes_head=True,ec='red',fc='red')
    
        
    # Add the circle to the axes
    ax.add_patch(marble)
    ax.add_patch(coilr)
    ax.add_patch(coill)
    
    if(zm!=0):
        ax.arrow(0,zm,0,Fz*5,width=0.5,head_width=1.5,head_length=2,length_includes_head=False,ec='black',fc='black')
    
    ax.set_aspect('equal', 'box')
    
    #plt.quiver(xi, yi, Bri, Bzi)
    #plt.clabel(contour, inline=1, fontsize=10)
    
    #ax.grid(1)
    
    plt.xlabel(r'$x/\mathrm{mm}$')
    plt.ylabel(r'$z/\mathrm{mm}$')
    plt.show()
    #plt.savefig("marble_-15mm_20A.png",dpi=300)
    plt.savefig("marble_"+ str(zm) + "mm_20A.png", dpi=300)
        
    plt.close(fig)