# -*- coding: utf-8 -*-
"""
Created on Sun Mar 30 07:53:02 2025

@author: Fabian Mink
"""


import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


def func(iz, Lmin, d, z0, zs): 
    i, z = iz 
    
    Eco = (0.5*Lmin*i**2)*(1+d*np.exp(-(((z-z0)/zs)**2)))
    
    return Eco
    

fem_data = np.genfromtxt('output_Ia_z.csv', delimiter=',')

Ia = fem_data[:,0]
Ib = fem_data[:,1]
z = fem_data[:,2] * 0.001  #mm -> m conversion
F = fem_data[:,4];
Eco = fem_data[:,5];


#0.5mm diameter Test parameters
Lmin =  1.29e-3;  #Inductance (for marble out of coil)
d =       0.396;  #Relative increase of inductance (for marble in middle of coil)
z0 =  -10.51e-3;  #m
zs =    9.60e-3;  #m 
pstart = [Lmin, d, z0, zs]


fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(Ia, z, Eco)


Ia_fit = np.arange(-10,10,0.5);
z_fit = np.arange(-40,30,1) * 1e-3;

Ia_FIT,z_FIT = np.meshgrid(Ia_fit, z_fit) 

# Curve fitting 
popt, pcov = curve_fit(func, (Ia, z), Eco, p0=pstart, bounds=([0.1e-3,0,-20e-3,1e-3],[10e-3,1,20e-3,20e-3])) 

Z_FIT = func((Ia_FIT,z_FIT), *popt) 

#ax.plot_surface(Ia_FIT,z_FIT, Z_FIT, color='red', alpha=0.5) 
ax.plot_surface(Ia_FIT,z_FIT, Z_FIT, alpha=0.5) 
ax.set_xlabel('X') 
ax.set_ylabel('Y') 
ax.set_zlabel('Z') 
plt.show()

  