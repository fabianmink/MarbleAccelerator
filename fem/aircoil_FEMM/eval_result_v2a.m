plottype = 0;

M = csvread('output_Ia_z.csv');
Ia = M(:,1);
Ib = M(:,2);
z = M(:,3);
F = M(:,5);
Eco = M(:,6);
Emag = M(:,7);

%Psia_from_Eco = [0; diff(Eco)./ (diff(Ia))];  
%La_from_Eco = [0; diff(Psia_from_Eco)./ (diff(Ia))];  
%F_from_Eco = [0; diff(Eco)./ (diff(z)*0.001)];  %z is in mm

%good fit (Matlab curveFitter):
%(c*x^2)*(1+d*exp(-(((y+g)/e)^2)))

%(c*Ia^2)*(1+d*exp(-(((z+g)/e)^2)))

c =   1.663e-05;
d =      0.4934; 
e =        9.27; 
g =       10.51; 

Ia_fit = -20:1:20;
z_fit  = -40:1:30;
[Ia_FIT,z_FIT] = meshgrid(Ia_fit, z_fit);

Eco_FIT = (c*Ia_FIT.^2) .* (1+d*exp(-(((z_FIT+g)/e).^2)));

%Multiplication *1000 is due to z in unit "mm" instead of "m"
F_FIT = 1000* (c*Ia_FIT.^2) .* (d*exp(-(((z_FIT+g)/e).^2)) ) .* (-2*(z_FIT+g)/e) * 1/e ;

Psi_FIT = 2*c*Ia_FIT .* (1+d*exp(-(((z_FIT+g)/e).^2)));
L_FIT = 2*c .* (1+d*exp(-(((z_FIT+g)/e).^2)));

if(plottype == 0)
scatter3(Ia, z, Eco);
hold on;
surf(Ia_FIT, z_FIT, Eco_FIT);
hold off;
zlabel('E_{co}/J')
end

if(plottype == 1)
scatter3(Ia, z, F);
hold on;
surf(Ia_FIT, z_FIT, F_FIT);
hold off;
zlabel('F_{z}/N')
end

if(plottype == 2)
surf(Ia_FIT, z_FIT, Psi_FIT);
zlabel('\Psi_a/Vs')
end

if(plottype == 3)
surf(Ia_FIT, z_FIT, L_FIT);
zlabel('L_a/H')
end

xlabel('I/A')
ylabel('z/mm')

