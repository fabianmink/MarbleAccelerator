# -*- coding: utf-8 -*-
"""
Created on Sun Mar 30 07:53:02 2025

@author: Fabian Mink
"""


import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy.optimize import curve_fit

plottype = 1

def magnetics_model(iz, Lmin, d, z0, zs): 
    i, z = iz
    
    #Coenergy
    Eco = (0.5*Lmin*i**2) * (1+d*np.exp(-0.5*(((z-z0)/zs)**2)))
    
    #Force
    F =   (0.5*Lmin*i**2) * (  d*np.exp(-0.5*(((z-z0)/zs)**2))) * (-1*(z-z0)/zs) * 1/zs

    #Flux linkage and inductance
    Psi = Lmin*i *          (1+d*np.exp(-0.5*(((z-z0)/zs)**2)))
    L =   Lmin *            (1+d*np.exp(-0.5*(((z-z0)/zs)**2)))

    #dPsi / dz
    dPsi_dz = Lmin*i *      (  d*np.exp(-0.5*(((z-z0)/zs)**2))) * (-1*(z-z0)/zs) * 1/zs;
    
    return (Eco, F, Psi, L, dPsi_dz)

def Eco_model(iz, Lmin, d, z0, zs):
    mm = magnetics_model(iz, Lmin, d, z0, zs)
    return mm[0]

def Fmax(i, Lmin, d, z0, zs): 
    
    Fmax = (0.5*Lmin*i**2) *  d*np.exp(-0.5) * 1/zs
    
    return Fmax

    
#fem_data = np.genfromtxt('output_Ia_z.csv', delimiter=',')
fem_data = np.genfromtxt('output_N180.csv', delimiter=',')


Ia = fem_data[:,0]
Ib = fem_data[:,1]
z = fem_data[:,2] * 0.001  #mm -> m conversion
F = fem_data[:,4];
Eco = fem_data[:,5];


#Start parameters
Lmin =   0.5e-3;  #Inductance (for marble out of coil)
d =         0.4;  #Relative increase of inductance (for marble in middle of coil)
z0 =   -10.0e-3;  #m
zs =     7.0e-3;  #m 
pstart = [Lmin, d, z0, zs]


# Curve fitting 
popt, pcov = curve_fit(Eco_model, (Ia, z), Eco, p0=pstart, bounds=([0.1e-3,0,-20e-3,1e-3],[10e-3,1,20e-3,20e-3])) 

Lmin, d, z0, zs = popt

Ia_fit = np.arange(-12,12,0.5);
z_fit = np.arange(-45,35,1) * 1e-3;

Ia_FIT,z_FIT = np.meshgrid(Ia_fit, z_fit) 

Eco_FIT, F_FIT, _, _, _ = magnetics_model((Ia_FIT,z_FIT), *popt) 

Fmax_fit = Fmax(Ia_fit, *popt)
#Fmax_FIT,z_FIT = np.meshgrid(Fmax_fit, z_fit)     
z_Fmax = Ia_fit*0 + z0 - zs;
z_Fmax_neg = Ia_fit*0 + z0 + zs;

fig = plt.figure()

if (plottype<10):
    ax = fig.add_subplot(projection='3d')
else:
    ax = fig.add_subplot()


if (plottype == 0):
    ax.scatter(Ia, z*1000, Eco)
    ax.plot_surface(Ia_FIT,z_FIT*1000, Eco_FIT, alpha=0.5) 
    ax.set_zlabel('$E_\mathrm{co} / \mathrm{Ws}$') 
    ax.set_xlabel('$I_\mathrm{a} / \mathrm{A}$') 
    ax.set_ylabel('$z / \mathrm{mm}$') 
    
if (plottype == 1):
    #ax.scatter(Ia, z*1000, F, marker='x', color='k')
    ax.plot(Ia, z*1000, F, 'k.', ms=4)
    ax.plot_surface(Ia_FIT,z_FIT*1000, F_FIT, alpha=0.5, cmap=cm.hsv) 
    ax.plot(Ia_fit,z_Fmax*1000,Fmax_fit,'r-')
    ax.plot(Ia_fit,z_Fmax_neg*1000,-Fmax_fit,'b-')
    ax.set_zlabel('$F_\mathrm{z} / \mathrm{N}$')
    ax.set_xlabel('$I_\mathrm{a} / A$') 
    ax.set_ylabel('$z / \mathrm{mm}$') 

if (plottype == 11):
    ax.plot(z_FIT*1000,F_FIT)
    #ax.plot(z_FIT*1000,Fmax_FIT)
    #ax.plot((z0-zs) * np.array([1, 1])*1000, np.array([-1, 1]))
    ax.set_xlabel('$z / \mathrm{mm}$') 
    ax.set_ylabel('$F_\mathrm{z} / \mathrm{N}$') 

if (plottype == 12):
    ax.plot(Ia_fit,Fmax_fit)
    #gravitation force
    ax.plot(np.array([min(Ia_fit), max(Ia_fit)]),0.0085*9.81* np.array([1, 1]))
    ax.grid()
    ax.set_xlabel('$I_\mathrm{a} / \mathrm{A}$') 
    ax.set_ylabel('$F_\mathrm{z,max} / \mathrm{N}$')     


ax.set_position([0,0,1,1])

plt.show()

  