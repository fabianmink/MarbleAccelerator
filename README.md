# Marble Accelerator
This project deals with the construction of various accelerators (coil guns) for metallic marbles. It might be used e.g. for the GraviTrax marble run.

## Coilgun Designs

### Horizontal coil design with two coils
![Horizontal coil design](img/coilgun_horizontal.jpg)

### Vertical coil design (single coil)

## FEM Simulation (FEMM)

![FEM Simulation](img/FEMM_example.png)


## Fitting of FEM results (Matlab)

The fit is based on the following coenergy equation:

- Co-Energy: $E_{\mathrm{Co}} = \frac{1}{2} \cdot L_\mathrm{min} \cdot I^2 \cdot \left[ 1 + d \cdot \exp \left( - \frac{z - z_0}{z_\mathrm{s}} \right)^2 \right]$

i.e.:

- z-Direction Force: $F_\mathrm{z} = \frac{\partial E_{\mathrm{Co}}}{\partial z} = - L_\mathrm{min} \cdot I^2 \cdot \left[ d \cdot \exp \left( - \frac{z - z_0}{z_\mathrm{s}} \right)^2 \right] \cdot \frac{z - z_0}{z^2_\mathrm{s}}$

- Flux Linkage: $\Psi = \frac{\partial E_{\mathrm{Co}}}{\partial I} = L_\mathrm{min} \cdot I \cdot \left[ 1 + d \cdot \exp \left( - \frac{z - z_0}{z_\mathrm{s}} \right)^2 \right]$

- Inductance: $L = \frac{\partial \Psi}{\partial I} = \frac{\partial^2 E_{\mathrm{Co}}}{\partial I^2} = L_\mathrm{min} \cdot \left[ 1 + d \cdot \exp \left( - \frac{z - z_0}{z_\mathrm{s}} \right)^2 \right]$

- Flux derivative: $\frac{\partial \Psi}{\partial z} = \frac{\partial^2 E_{\mathrm{Co}}}{\partial I \partial z} = -2 \cdot L_\mathrm{min} \cdot I \cdot \left[ d \cdot \exp \left( - \frac{z - z_0}{z_\mathrm{s}} \right)^2 \right] \cdot \frac{z - z_0}{z^2_\mathrm{s}}$ (needed for induced voltage calculation)



![Force Fit](img/force_fitting.png)

## Dynamic System Simulation Model

The dynamic simulation implements the following equations:

- Velocity (z-direction): ${\dot{v}}_{\mathrm{z}} = \frac{1}{m} \cdot F_{\mathrm{z}}$
- Velocity (z-direction): $\dot{v} = \frac{1}{m} \cdot F_\mathrm{z}$
- z-Position: $\dot{z} = v_\mathrm{z}$
- Force calculation: $F_\mathrm{z} = F_\mathrm{z}\left(I, z\right)$ (see equation in chapter above)
- Coil current: $\dot{I} = \frac{1}{L \left(I, z\right) } \cdot \left[ u - R \cdot I - u_\mathrm{ind} \right]$
- Induced voltage: $u_\mathrm{ind} = \frac{\partial \Psi}{\partial z} \cdot v_\mathrm{z}$


![System Simulation](img/simulink_model.png)
