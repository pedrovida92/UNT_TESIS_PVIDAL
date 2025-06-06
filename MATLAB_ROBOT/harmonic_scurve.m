clc, clear
tic

t7 = 4;

t = linspace(0,t7,t7*100);
r0 = 0;
rf = 1;

cv = 0.5;

Vmin = (rf-r0)/t7;
V = Vmin*(1+cv);

t3 = t7*(1-1/(cv+1));

ca = 0.2;
Amin = V/t3;
A = Amin*(1+ca);

t1 = t3*(1-1/(ca+1));

t2 = t3-t1;
t4 = t7-t3;
t5 = t4+t1;
t6 = t7-t1;

J = 2*A/t1;
w = 2*pi/t1;

t01 = t(t<=t1);
t12 = t((t>t1)&(t<=t2));
t23 = t((t>t2)&(t<=t3));
t34 = t((t>t3)&(t<=t4));
t45 = t((t>t4)&(t<=t5));
t56 = t((t>t5)&(t<=t6));
t67 = t(t>t6);

t = [t01,t12,t23,t34,t45,t56,t67];

j1 = (J/2)*(1-cos(w*t01));
a1 = (J/2)*(t01-(sin(w*t01)/w));
v1 = (J/2)*(t01.^2/2 + (cos(w*t01)-1)/w^2);
r1 = r0 + (J/2)*(t01.^3/6 - t01/w^2 + sin(w*t01)/w^3);

j2 = zeros(size(t12));
a2 = A*ones(size(t12));
v2 = A*(t12-t1/2);
r2 = r0 + (A/2)*(t12.^2 - t1*t12 + t1^2/3 - 2/w^2);

j3 = (-J/2)*(1-cos(w*(t23-t2)));
a3 = A-(J/2)*(t23-t2 - sin(w*(t23-t2))/w);
v3 = A*(t23-t1/2) - (J/2)*((t23-t2).^2/2 + (cos(w*(t23-t2))-1)/w^2);
r3 = r0 + A*(t23.^2/2 - t1*t23/2 + t1^2/6 - 1/w^2) -(J/2)*((t23-t2).^3/6 + sin(w*(t23-t2))/w^3 - (t23-t2)/w^2);

j4 = zeros(size(t34));
a4 = zeros(size(t34));
v4 = V*ones(size(t34));
r4 = r0 + V*(t34-t3/2);

j5 = -(J/2)*(1-cos(w*(t45-t4)));
a5 = -(J/2)*(t45-t4 - sin(w*(t45-t4))/w);
v5 = V - (J/2)*((t45-t4).^2/2 + (cos(w*(t45-t4))-1)/w^2);
r5 = r0 + V*(t45-t3/2) - (J/2)*((t45-t4).^3/6 + sin(w*(t45-t4))/w^3 - (t45-t4)/w^2);

j6 = zeros(size(t56));
a6 = -A*ones(size(t56));
v6 = V - A*(t56 + t1/2 -t5);
r6 = r0 + V*(t56 - t3/2) - A*((t56-t5).^2/2 + t1*(t56-t5)/2+t1^2/6-1/w^2);

j7 = (J/2)*(1-cos(w*(t67-t6)));
a7 = (J/2)*(t67-t6-sin(w*(t67-t6))/w)-A;
v7 = V-A*(t67+t1/2-t5)+(J/2)*((t67-t6).^2/2 + (cos(w*(t67-t6))-1)/w^2);
r7 = rf + V*(t67-t7) - A*((t67.^2-t7^2)/2+(t1/2-t5)*(t67-t7)) +(J/2)*((t67-t6).^3/6 + sin(w*(t67-t6))/w^3 - (t67-t7)/w^2 - t1^3/6);

j = [j1,j2,j3,j4,j5,j6,j7];
a = [a1,a2,a3,a4,a5,a6,a7];
v = [v1,v2,v3,v4,v5,v6,v7];
r = [r1,r2,r3,r4,r5,r6,r7];

toc

plot(t,j,t,a,t,v,t,r,'LineWidth',2)
