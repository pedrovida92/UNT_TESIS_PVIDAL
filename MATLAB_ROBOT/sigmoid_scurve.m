% Sigmoid S-Curve
clc, clear


% Ts = t1;
% Tj = t2-t1;
% Ta = t4-t3;
% Tv = t8-t7;

r0 = 0;
rf = 10000;
D = rf-r0;

Smax = 300;
Jmax = 100;
Amax = 80;
Vmax = 200;

Ts = sqrt(3)*Jmax/Smax;
Tj = Amax/Jmax - Ts;
Ta = Vmax/Amax - 2*Ts - Tj;
Tv = abs(D)/Vmax - (4*Ts+2*Tj+Ta);

t0 = 0;
t1 = Ts;
t2 = t1+Tj;
t3 = t2+Ts;
t4 = t3+Ta;
t5 = t4+Ts;
t6 = t5+Tj;
t7 = t6+Ts;
t8 = t7+Tv;
t9 = t8+Ts;
t10 = t9+Tj;
t11 = t10+Ts;
t12 = t11+Ta;
t13 = t12+Ts;
t14 = t13+Tj;
t15 = t14+Ts;

t = linspace(t0,t15,100*t15);

t0_1 = t(t<=t1);
t1_2 = t((t>t1)&(t<=t2));
t2_3 = t((t>t2)&(t<=t3));
t3_4 = t((t>t3)&(t<=t4));
t4_5 = t((t>t4)&(t<=t5));
t5_6 = t((t>t5)&(t<=t6));
t6_7 = t((t>t6)&(t<=t7));
t7_8 = t((t>t7)&(t<=t8));
t8_9 = t((t>t8)&(t<=t9));
t9_10 = t((t>t9)&(t<=t10));
t10_11 = t((t>t10)&(t<=t11));
t11_12 = t((t>t11)&(t<=t12));
t12_13 = t((t>t12)&(t<=t13));
t13_14 = t((t>t13)&(t<=t14));
t14_15 = t(t>t14);

%Jmax = 1;

tau1 = (t0_1-t0)/(t1-t0);
tau2 = (t2_3-t2)/(t3-t2);
tau3 = (t4_5-t4)/(t5-t4);
tau4 = (t6_7-t6)/(t7-t6);
tau5 = (t8_9-t8)/(t9-t8);
tau6 = (t10_11-t10)/(t11-t10);
tau7 = (t12_13-t12)/(t13-t12);
tau8 = (t14_15-t14)/(t15-t14);

a = sqrt(3)/2;

n1 = length(t0_1);

j1 = zeros(1,n1);
a1 = zeros(1,n1);
v1 = zeros(1,n1);
r1 = zeros(1,n1);

a_sum = 0;
v_sum = 0;
r_sum = r0;

for i=1:n1
    j1(i) = Jmax*(1/(1+exp(-a*(1/(1-tau1(i))-1/tau1(i)))));
    a_sum = a_sum + j1(i)/n1;
    a1(i) = a_sum;
    v_sum = v_sum + a1(i)/n1;
    v1(i) = v_sum;
    r_sum = r_sum + v1(i)/n1;
    r1(i) = r_sum;
end

n2 = length(t1_2);

j2 = zeros(1,n2);
a2 = zeros(1,n2);
v2 = zeros(1,n2);
r2 = zeros(1,n2);

for i=1:n2
    j2(i) = Jmax;
    a_sum = a_sum + j2(i)/n2;
    a2(i) = a_sum;
    v_sum = v_sum + a2(i)/n2;
    v2(i) = v_sum;
    r_sum = r_sum + v2(i)/n2;
    r2(i) = r_sum;
end

n3 = length(t2_3);

j3 = zeros(1,n3);
a3 = zeros(1,n3);
v3 = zeros(1,n3);
r3 = zeros(1,n3);

for i=1:n3
    j3(i) = Jmax*(1/(1+exp(a*(1/(1-tau2(i))-1/tau2(i)))));
    a_sum = a_sum + j3(i)/n3;
    a3(i) = a_sum;
    v_sum = v_sum + a3(i)/n3;
    v3(i) = v_sum;
    r_sum = r_sum + v3(i)/n3;
    r3(i) = r_sum;
end

n4 = length(t3_4);

j4 = zeros(1,n4);
a4 = zeros(1,n4);
v4 = zeros(1,n4);
r4 = zeros(1,n4);

%Amax = Jmax*(Ts+Tj);
a_sum = Amax;

for i=1:n4
    j4(i) = 0;
    a4(i) = Amax;
    v_sum = v_sum + a4(i)/n4;
    v4(i) = v_sum;
    r_sum = r_sum + v4(i)/n4;
    r4(i) = r_sum;
end

n5 = length(t4_5);

j5 = zeros(1,n5);
a5 = zeros(1,n5);
v5 = zeros(1,n5);
r5 = zeros(1,n5);

for i=1:n5
    j5(i) = -Jmax*(1/(1+exp(-a*(1/(1-tau3(i))-1/tau3(i)))));
    a_sum = a_sum + j5(i)/n5;
    a5(i) = a_sum;
    v_sum = v_sum + a5(i)/n5;
    v5(i) = v_sum;
    r_sum = r_sum + v5(i)/n5;
    r5(i) = r_sum;
end

n6 = length(t5_6);

j6 = zeros(1,n6);
a6 = zeros(1,n6);
v6 = zeros(1,n6);
r6 = zeros(1,n6);

for i=1:n6
    j6(i) = -Jmax;
    a_sum = a_sum + j6(i)/n6;
    a6(i) = a_sum;
    v_sum = v_sum + a6(i)/n6;
    v6(i) = v_sum;
    r_sum = r_sum + v6(i)/n6;
    r6(i) = r_sum;
end

n7 = length(t6_7);

j7 = zeros(1,n7);
a7 = zeros(1,n7);
v7 = zeros(1,n7);
r7 = zeros(1,n7);

for i=1:n7
    j7(i) = -Jmax*(1/(1+exp(a*(1/(1-tau4(i))-1/tau4(i)))));
    a_sum = a_sum + j7(i)/n7;
    a7(i) = a_sum;
    v_sum = v_sum + a7(i)/n7;
    v7(i) = v_sum;
    r_sum = r_sum + v7(i)/n7;
    r7(i) = r_sum;
end

n8 = length(t7_8);

j8 = zeros(1,n8);
a8 = zeros(1,n8);
v8 = zeros(1,n8);
r8 = zeros(1,n8);

%Vmax = Amax*(2*Ts+Tj+Ta);
v_sum = Vmax;

for i=1:n8
    j8(i) = 0;
    a8(i) = 0;
    v8(i) = Vmax;
    r_sum = r_sum + v8(i)/n8;
    r8(i) = r_sum;
end

n9 = length(t8_9);

j9 = zeros(1,n9);
a9 = zeros(1,n9);
v9 = zeros(1,n9);
r9 = zeros(1,n9);

for i=1:n9
    j9(i) = -Jmax*(1/(1+exp(-a*(1/(1-tau5(i))-1/tau5(i)))));
    a_sum = a_sum + j9(i)/n9;
    a9(i) = a_sum;
    v_sum = v_sum + a9(i)/n9;
    v9(i) = v_sum;
    r_sum = r_sum + v9(i)/n9;
    r9(i) = r_sum;
end

n10 = length(t9_10);

j10 = zeros(1,n10);
a10 = zeros(1,n10);
v10 = zeros(1,n10);
r10 = zeros(1,n10);

for i=1:n10
    j10(i) = -Jmax;
    a_sum = a_sum + j10(i)/n10;
    a10(i) = a_sum;
    v_sum = v_sum + a10(i)/n10;
    v10(i) = v_sum;
    r_sum = r_sum + v10(i)/n10;
    r10(i) = r_sum;
end

n11 = length(t10_11);

j11 = zeros(1,n11);
a11 = zeros(1,n11);
v11 = zeros(1,n11);
r11 = zeros(1,n11);

for i=1:n11
    j11(i) = -Jmax*(1/(1+exp(a*(1/(1-tau6(i))-1/tau6(i)))));
    a_sum = a_sum + j11(i)/n11;
    a11(i) = a_sum;
    v_sum = v_sum + a11(i)/n11;
    v11(i) = v_sum;
    r_sum = r_sum + v11(i)/n11;
    r11(i) = r_sum;
end

n12 = length(t11_12);

j12 = zeros(1,n12);
a12 = zeros(1,n12);
v12 = zeros(1,n12);
r12 = zeros(1,n12);

a_sum = -Amax;

for i=1:n12
    j12(i) = 0;
    a12(i) = -Amax;
    v_sum = v_sum + a12(i)/100;
    v12(i) = v_sum;
    r_sum = r_sum + v12(i)/100;
    r12(i) = r_sum;
end

n13 = length(t12_13);

j13 = zeros(1,n13);
a13 = zeros(1,n13);
v13 = zeros(1,n13);
r13 = zeros(1,n13);

for i=1:n13
    j13(i) = Jmax*(1/(1+exp(-a*(1/(1-tau7(i))-1/tau7(i)))));
    a_sum = a_sum + j13(i)/n13;
    a13(i) = a_sum;
    v_sum = v_sum + a13(i)/n13;
    v13(i) = v_sum;
    r_sum = r_sum + v13(i)/n13;
    r13(i) = r_sum;
end

n14 = length(t13_14);

j14 = zeros(1,n14);
a14 = zeros(1,n14);
v14 = zeros(1,n14);
r14 = zeros(1,n14);

for i=1:n14
    j14(i) = Jmax;
    a_sum = a_sum + j14(i)/n14;
    a14(i) = a_sum;
    v_sum = v_sum + a14(i)/n14;
    v14(i) = v_sum;
    r_sum = r_sum + v14(i)/n14;
    r14(i) = r_sum;
end

n15 = length(t14_15);

j15 = zeros(1,n15);
a15 = zeros(1,n15);
v15 = zeros(1,n15);
r15 = zeros(1,n15);

R = Vmax*(4*Ts+2*Tj+Ta+Tv);

for i=1:n15
    j15(i) = Jmax*(1/(1+exp(a*(1/(1-tau8(i))-1/tau8(i)))));
    a_sum = a_sum + j15(i)/n15;
    a15(i) = a_sum;
    v_sum = v_sum + a15(i)/n15;
    v15(i) = v_sum;
    r_sum = r_sum + v15(i)/n15;
    r15(i) = r_sum;
end

%j1 = Jmax*(1./(1+exp(-a*(1./(1-tau)-1./tau))));

t = [t0_1,t1_2,t2_3,t3_4,t4_5,t5_6,t6_7,t7_8,t8_9,t9_10,t10_11,t11_12,t12_13,t13_14,t14_15];
j = [j1,j2,j3,j4,j5,j6,j7,j8,j9,j10,j11,j12,j13,j14,j15];
a = [a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15];
v = [v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14,v15];
r = [r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15];

%plot(t,j,t,a,t,v,t,r,'LineWidth',2)

figure()
subplot(4,1,1)
plot(t,j,'LineWidth',1)
subplot(4,1,2)
plot(t,a,'LineWidth',1)
subplot(4,1,3)
plot(t,v,'LineWidth',1)
subplot(4,1,4)
plot(t,r,'LineWidth',1)
