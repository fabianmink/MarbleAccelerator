# -*- coding: utf-8 -*-
"""
Created on Sun Mar 30 07:53:02 2025

@author: Fabian Mink
"""


import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

plottype = 0

def magnetics_model(iz, Lmin, d, z0, zs): 
    i, z = iz 
    
    #Coenergy
    Eco = (0.5*Lmin*i**2) * (1+d*np.exp(-(((z-z0)/zs)**2)))
    
    #Force
    F =   (0.5*Lmin*i**2) * (  d*np.exp(-(((z-z0)/zs)**2))) * (-2*(z-z0)/zs) * 1/zs

    #Flux linkage and inductance
    Psi = Lmin*i *          (1+d*np.exp(-(((z-z0)/zs)**2)))
    L =   Lmin *            (1+d*np.exp(-(((z-z0)/zs)**2)))

    #dPsi / dz
    dPsi_dz = Lmin*i *      (  d*np.exp(-(((z-z0)/zs)**2))) * (-2*(z-z0)/zs) * 1/zs;
    
    return (Eco, F, Psi, L, dPsi_dz)

def Eco_model(iz, Lmin, d, z0, zs):
    mm = magnetics_model(iz, Lmin, d, z0, zs)
    return mm[0]

    
#fem_data = np.genfromtxt('output_Ia_z.csv', delimiter=',')
fem_data = np.genfromtxt('output_N180.csv', delimiter=',')


Ia = fem_data[:,0]
Ib = fem_data[:,1]
z = fem_data[:,2] * 0.001  #mm -> m conversion
F = fem_data[:,4];
Eco = fem_data[:,5];


#0.5mm diameter Test / Start parameters
Lmin =  1.29e-3;  #Inductance (for marble out of coil)
d =       0.396;  #Relative increase of inductance (for marble in middle of coil)
z0 =  -10.51e-3;  #m
zs =    9.60e-3;  #m 
pstart = [Lmin, d, z0, zs]


# Curve fitting 
popt, pcov = curve_fit(Eco_model, (Ia, z), Eco, p0=pstart, bounds=([0.1e-3,0,-20e-3,1e-3],[10e-3,1,20e-3,20e-3])) 



Ia_fit = np.arange(-10,10,0.5);
z_fit = np.arange(-40,30,1) * 1e-3;

Ia_FIT,z_FIT = np.meshgrid(Ia_fit, z_fit) 

Eco_FIT, F_FIT, _, _, _ = magnetics_model((Ia_FIT,z_FIT), *popt) 

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

if (plottype == 0):
    ax.scatter(Ia, z*1000, Eco)
    ax.plot_surface(Ia_FIT,z_FIT*1000, Eco_FIT, alpha=0.5) 
    ax.set_zlabel('$E_\mathrm{co} / Ws$') 
    
if (plottype == 1):
    ax.scatter(Ia, z*1000, F)
    ax.plot_surface(Ia_FIT,z_FIT*1000, F_FIT, alpha=0.5) 
    ax.set_zlabel('$F_z / N$') 


ax.set_xlabel('$I_\mathrm{a} / A$') 
ax.set_ylabel('$z / mm$') 

plt.show()

  