clc, clear

M = readmatrix("ATC_LOAD.csv");
n = 10;
m = length(M)/n;
t = linspace(0,0.01*m,m);

sum_i = zeros(m,3);

for i=1:n
    sum_i = sum_i + M(1+m*(i-1):i*m,7:9);
end

Pi_mean = sum_i/n;

lij = zeros(m,n);

for j=1:n
    for i=1:m
        lij(i,j) = norm(M(i+(j-1)*m,7:9)-Pi_mean(i,:));
    end
end

 li = mean(lij,2);

 Sli = zeros(m,1);

 sum_sli = 0;

 for i = 1:m
     for j = 1:n
         sum_sli = sum_sli + (lij(i,j)-li(i))^2;
     end
     Sli(i) = sqrt(sum_sli/(n-1));
     sum_sli = 0;
 end

 RT_pi = zeros(m,1);

 for i = 1:m
     RT_pi(i) = li(i)+3*Sli(i);
 end

 RT_max = max(RT_pi);

 plot(t,RT_pi)
 grid on
