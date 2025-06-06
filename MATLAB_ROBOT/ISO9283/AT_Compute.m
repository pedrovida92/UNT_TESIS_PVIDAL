clc, clear, close all 

M = readmatrix("ATL_test1.csv");
n = 10;
m = length(M)/n;
t = linspace(0,0.01*m,m);

sum_ci = zeros(m,3);
sum_i = zeros(m,3);

for i=1:n
    sum_ci = sum_ci + M(1+m*(i-1):i*m,1:3);
    sum_i = sum_i + M(1+m*(i-1):i*m,7:9);

end

Pci_mean = sum_ci/n;
Pi_mean = sum_i/n;

ATP = zeros(m,1);

for i=1:m
    ATP(i,1) = norm(Pi_mean(i,:)-Pci_mean(i,:));
end

ATP_max = max(ATP);

font_size = 12;

figure()
plot3(Pci_mean(:,1),Pci_mean(:,2),Pci_mean(:,3),Pi_mean(:,1),Pi_mean(:,2),Pi_mean(:,3),LineWidth=1.5)
legend({'Ruta de Comando','Ruta de Baricentro'},'Location','northeast','FontSize',font_size)
xlabel('$x$ $[mm]$','Interpreter','latex','FontSize',font_size)
ylabel('$y$ $[mm]$','Interpreter','latex','FontSize',font_size)
zlabel('$z$ $[mm]$','Interpreter','latex','FontSize',font_size)
%xlim([200,600])
%ylim([-200,200])
%zlim([150,550])
%view([30 30])
grid on


figure()
plot(t,ATP,LineWidth=1.5)
xlim([0,t(end)]);
xlabel('tiempo [$s$]','Interpreter','latex','FontSize',font_size)
ylabel('$ATP_P$ $[mm]$','Interpreter','latex','FontSize',font_size)
grid on