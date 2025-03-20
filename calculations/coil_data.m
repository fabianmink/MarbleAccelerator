l_coil = 15e-3;  %m Coil length
r_i = 8e-3;      %m Coil inner radius
r_o = 14e-3;     %m Coil outer radius
A_coil = l_coil * (r_o-r_i);  %m^2 coil cross section (1 side)
d_wire = [0.75e-3 0.5e-3];    %m Wire diameter

r_wire = d_wire/2;    %wire radius
A_wire = pi*r_wire.^2; %wire cross section

cu_fill = 0.65; %Copper filling factor

N = A_coil*cu_fill./A_wire;  %Number of turns


mu0 = 1.256e-6;
r_avg = (r_o + r_i)/2;

%L_simple = N.^2 * mu0 * pi*r_avg^2/l_coil;
%L_adv = N.^2 * mu0 * pi*r_avg^2/(l_coil + 0.9*r_avg);

%Harold A. Wheeler, "Simple Inductance Formulas for Radio Coils," Proceedings of the I.R.E., October 1928, pp. 1398-1400. 
%L {uH} = 31.6 * N^2 * r1^2 / (6*r1 + 9*L + 10*(r2-r1))
%L{uH} = Inductance in microHenries
%N^2 = Total Number of turns on coil Squared
%r1 = Radius of the inside of the coil {meters}
%r2 = Radius of the outside of the coil {meters}
%L = Length of the coil {meters}
L_wheeler = 31.6e-6 * N.^2 * r_i^2 / (6*r_i + 9*l_coil + 10*(r_o-r_i));
l_wire = 2*pi*r_avg*N;

rhoCu = 0.019; %Ohm*mm^2/l
R = rhoCu * l_wire ./ (A_wire*10^6);  %Ohm coil Resistance

L = L_wheeler;
tau = L./R;  %s time constant of coil 1st order System

