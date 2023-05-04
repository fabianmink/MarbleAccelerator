N = 50;
mu0 = 1.256e-6;
l = 15e-3;
R = 10e-3; %average

%L = N^2 * mu0 * pi*R^2/l;
L = mu0 * N^2 * R^2*pi/(l + 0.9*R);