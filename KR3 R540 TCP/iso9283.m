% cubo iso 9238

clc, clear

L = 300;

x0 = 250;
y0 = 0;
z0 = 200;

P2 = [x0+L, y0+L/2, z0+L];
P3 = [x0+L, y0-L/2, z0+L];
P4 = [x0, y0-L/2, z0];
P5 = [x0, y0+L/2, z0];

P1 = (P2+P4)/2;
P6 = (P2+P5)/2;
P9 = (P3+P4)/2;
P7 = (P1+P6)/2;
P8 = (P1+P9)/2;

u_r1 = (P2-P6)/norm(P2-P6);
r_d = 150;
r1 = r_d*u_r1;
P69 = P1 + r1;

u_r2 = (P5-P6)/norm(P5-P6);
r2 = r_d*u_r2;
P96 = P1 - r1;

p69 = P1 + 0.1*r1;
r96 = P1 + 0.1*r2;



P = [1, P1; 2, P2; 3, P3; 4, P4; 5, P5; 6, P6; 7, P7; 8, P8; 9, P9]