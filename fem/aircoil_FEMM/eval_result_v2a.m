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
%(0.5*a*x^2 )*(1+b*exp(-(((y-c)/d)^2)))

%(0.5*Lmin*Ia^2)*(1+d*exp(-(((z-z0)/z_s)^2)))

%1mm diameter
%Lmin = 3.3260e-05;  %Inductance (for marble out of coil)
%d =        0.4934;  %Relative increase of inductance (for marble in middle of coil)
%zs =      9.27e-3;  %m 
%z0 =    -10.51e-3;  %m

%0.5mm diameter
Lmin =  1.29e-3;  %Inductance (for marble out of coil)
d =       0.396;  %Relative increase of inductance (for marble in middle of coil)
zs =    9.60e-3;  %m 
z0 =  -10.51e-3;  %m

Ia_fit = -10:0.5:10;
%Ia_fit = [2 2.5];
z_fit  = (-40:1:30)*1e-3;
[Ia_FIT,z_FIT] = meshgrid(Ia_fit, z_fit);

%Coenergy
Eco_FIT = (0.5*Lmin*Ia_FIT.^2) .* (1+d*exp(-(((z_FIT-z0)/zs).^2)));

%Force
F_FIT =   (0.5*Lmin*Ia_FIT.^2) .* (d*exp(-(((z_FIT-z0)/zs).^2))) .* (-2*(z_FIT-z0)/zs) * 1/zs ;

%Flux linkage and inductance
Psi_FIT = Lmin*Ia_FIT .* (1+d*exp(-(((z_FIT-z0)/zs).^2)));
L_FIT =          Lmin .* (1+d*exp(-(((z_FIT-z0)/zs).^2)));

%dPsi / dz
dPsi_dz_FIT = Lmin*Ia_FIT .* (d*exp(-(((z_FIT-z0)/zs).^2))) .* (-2*(z_FIT-z0)/zs) * 1/zs;

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
zlabel('d\Psi / dz  V/(m/s)')
end

xlabel('I/A')
ylabel('z/mm')

