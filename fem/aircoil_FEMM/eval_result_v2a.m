plottype = 1;

M = csvread('output_Ia_z.csv');
Ia = M(:,1);
Ib = M(:,2);
z = M(:,3) * 0.001; %z Position mm -> m conversion
F = M(:,5);
Eco = M(:,6);
Emag = M(:,7);

%Psia_from_Eco = [0; diff(Eco)./ (diff(Ia))];  
%La_from_Eco = [0; diff(Psia_from_Eco)./ (diff(Ia))];  
%F_from_Eco = [0; diff(Eco)./ (diff(z))]; 

% good fit (Matlab curveFitter):
%(c*x^2)*(1+d*exp(-(((y+g)/e)^2)))
%(c*Ia^2)*(1+d*exp(-(((z+g)/e)^2)))

L = 3.3260e-05;   %Inductance (for marble out of coil)
d =      0.4934;  %Relative increase of inductance (for marble in middle of coil)
z_s =   9.27e-3;  %m 
z0 =  -10.51e-3;  %m

Ia_fit = -20:1:20;
%Ia_fit = [20 20];
z_fit  = (-40:1:30)*1e-3;
[Ia_FIT,z_FIT] = meshgrid(Ia_fit, z_fit);

%Coenergy
Eco_FIT = (0.5*L*Ia_FIT.^2) .* (1+d*exp(-(((z_FIT-z0)/z_s).^2)));

%Force
F_FIT =   (0.5*L*Ia_FIT.^2) .* (d*exp(-(((z_FIT-z0)/z_s).^2))) .* (-2*(z_FIT-z0)/z_s) * 1/z_s ;

%Flux linkage and inductance
Psi_FIT = L*Ia_FIT .* (1+d*exp(-(((z_FIT-z0)/z_s).^2)));
L_FIT =          L .* (1+d*exp(-(((z_FIT-z0)/z_s).^2)));

%dPsi / dz
dPsi_dz_FIT = L*Ia_FIT .* (1+d*exp(-(((z_FIT-z0)/z_s).^2))) .* (-2*(z_FIT-z0)/z_s) * 1/z_s;

if(plottype == 0)
scatter3(Ia, z*1000, Eco);
hold on;
surf(Ia_FIT, z_FIT*1000, Eco_FIT);
hold off;
zlabel('E_{co}/J')
end

if(plottype == 1)
scatter3(Ia, z*1000, F);
hold on;
surf(Ia_FIT, z_FIT*1000, F_FIT);
hold off;
zlabel('F_{z}/N')
end

if(plottype == 2)
surf(Ia_FIT, z_FIT*1000, Psi_FIT);
zlabel('\Psi_a/Vs')
end

if(plottype == 3)
surf(Ia_FIT, z_FIT*1000, L_FIT*1000);
zlabel('L_a/mH')
end


if(plottype == 4)
surf(Ia_FIT, z_FIT*1000, dPsi_dz_FIT);
zlabel('d\Psi / dz  Vs/m')
end

xlabel('I/A')
ylabel('z/mm')

