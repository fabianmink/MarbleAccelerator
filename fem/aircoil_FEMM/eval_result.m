load('output.csv')
I = output(:,1);
ky = output(:,2);
Fy = output(:,4);
Eco = output(:,5);

scatter3(I,ky, Fy)

scatter3(I,ky, Eco)
