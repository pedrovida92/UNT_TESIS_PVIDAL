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

%P1(1,:) = P1(2,:);

P1m = mean(P1);
P2m = mean(P2);
P3m = mean(P3);
P4m = mean(P4);
P5m = mean(P5);

AP1 = sqrt((P1m(1)-pc1(1))^2+(P1m(2)-pc1(2))^2+(P1m(3)-pc1(3))^2);
AP2 = sqrt((P2m(1)-pc2(1))^2+(P2m(2)-pc2(2))^2+(P2m(3)-pc2(3))^2);
AP3 = sqrt((P3m(1)-pc3(1))^2+(P3m(2)-pc3(2))^2+(P3m(3)-pc3(3))^2);
AP4 = sqrt((P4m(1)-pc4(1))^2+(P4m(2)-pc4(2))^2+(P4m(3)-pc4(3))^2);
AP5 = sqrt((P5m(1)-pc5(1))^2+(P5m(2)-pc5(2))^2+(P5m(3)-pc5(3))^2);





