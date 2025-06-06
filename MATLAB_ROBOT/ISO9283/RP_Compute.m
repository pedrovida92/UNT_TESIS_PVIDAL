clc, clear

M = readmatrix("AP_LOAD.csv");

P1 = zeros(30,6);
P2 = zeros(30,6);
P3 = zeros(30,6);
P4 = zeros(30,6);
P5 = zeros(30,6);

pc1 = [400,0,350];
pc2 = [550,150,500];
pc3 = [550,-150,500];
pc4 = [250,-150,200];
pc5 = [250,150,200];

k = 1;

for i=1:30
    P1(i,:) = M(k,:);
    k = k+1;
    P5(i,:) = M(k,:);
    k = k+1;
    P4(i,:) = M(k,:);
    k = k+1;
    P3(i,:) = M(k,:);
    k = k+1;
    P2(i,:) = M(k,:);
    k = k+1;
end

P1(1,:) = P1(2,:);

P1m = mean(P1);
P2m = mean(P2);
P3m = mean(P3);
P4m = mean(P4);
P5m = mean(P5);

l1 = zeros(30,1);
l2 = zeros(30,1);
l3 = zeros(30,1);
l4 = zeros(30,1);
l5 = zeros(30,1);

for j=1:30
    l1(j) = sqrt((P1(j,1)-P1m(1))^2+(P1(j,2)-P1m(2))^2+(P1(j,3)-P1m(3))^2);
    l2(j) = sqrt((P2(j,1)-P2m(1))^2+(P2(j,2)-P2m(2))^2+(P2(j,3)-P2m(3))^2);
    l3(j) = sqrt((P3(j,1)-P3m(1))^2+(P3(j,2)-P3m(2))^2+(P3(j,3)-P3m(3))^2);
    l4(j) = sqrt((P4(j,1)-P4m(1))^2+(P4(j,2)-P4m(2))^2+(P4(j,3)-P4m(3))^2);
    l5(j) = sqrt((P5(j,1)-P5m(1))^2+(P5(j,2)-P5m(2))^2+(P5(j,3)-P5m(3))^2);
end

l1m = mean(l1);
l2m = mean(l2);
l3m = mean(l3);
l4m = mean(l4);
l5m = mean(l5);

sum_Sl1 = 0;
sum_Sl2 = 0;
sum_Sl3 = 0;
sum_Sl4 = 0;
sum_Sl5 = 0;

for j=1:30
    sum_Sl1 = sum_Sl1 + (l1(j)-l1m)^2;
    sum_Sl2 = sum_Sl2 + (l2(j)-l2m)^2;
    sum_Sl3 = sum_Sl3 + (l3(j)-l3m)^2;
    sum_Sl4 = sum_Sl4 + (l4(j)-l4m)^2;
    sum_Sl5 = sum_Sl5 + (l5(j)-l5m)^2;
end

Sl1 = sqrt(sum_Sl1/(30-1));
Sl2 = sqrt(sum_Sl2/(30-1));
Sl3 = sqrt(sum_Sl3/(30-1));
Sl4 = sqrt(sum_Sl4/(30-1));
Sl5 = sqrt(sum_Sl5/(30-1));

RP1 = l1m + 3*Sl1;
RP2 = l2m + 3*Sl2;
RP3 = l3m + 3*Sl3;
RP4 = l4m + 3*Sl4;
RP5 = l5m + 3*Sl5;