M = csvread('output.csv');
y = M(:,2);
F = M(:,4);
Eco = M(:,5);
Emag = M(:,6);

F_from_Eco = [0; diff(Eco)./ (diff(y)*0.001)];  %y is in mm


%test
subplot(2,1,1)
plot(y,Eco,'b-','LineWidth',2)
hold on;
plot(y,Emag,'r-','LineWidth',1)
hold off;
ylabel('E/Ws')
xlabel('y/mm')

subplot(2,1,2)
plot(y,F,'b-','LineWidth',2)
hold on;
plot(y,F_from_Eco,'r-','LineWidth',1)
hold off;
ylabel('F/N')
xlabel('y/mm')

%Modell (Kloss)
y0 = -6;
yk = 5;
Fk = 1.51;
Fmod = -Fk * 2./( (y-y0)./yk  +  yk./(y-y0)  );

% plot(y,F,'b-','LineWidth',2)
% hold on;
% plot(y+12,F,'r-','LineWidth',2)
% %plot(-y,-F,'g-','LineWidth',2)
% plot(y,Fmod,'g--','LineWidth',1)
% hold off



clear myDim;
%Abmaße
myDim.x_cm = 16;
myDim.y_cm = 8.5;

%Ursprung KOS
myDim.x_cm_orig = 11; 
myDim.y_cm_orig = 3.5;

%Minimum KOS
myDim.x_cm_min = 0.375;
myDim.y_cm_min = 0.375;

%Maximum KOS
myDim.x_cm_max = 14;
myDim.y_cm_max = 7.0;

myDim.y_scale = 0.5;
myDim.x_scale = 2.0;

myDim.y_tickres = 0.5;
myDim.x_tickres = 2.0;

%myDim.x_noscale = 1;
%myDim.y_noscale = 1;

myDim.x_label = '{\it{y}} / mm';
myDim.y_label = '{\it{F}}_y / N';


ah = gca();
%drawPaper(myDim,'y_Fy',ah);