function grapher_gzy(M,Tz,Ty)

t = M(:,1);
q = rad2deg(M(:,2:7));
dq = rad2deg(M(:,8:13));
ddq = rad2deg(M(:,14:19));
dddq = rad2deg(M(:,20:25));

font_size = 12;
LW = 1.5;

figure('Position',[0 0 800 900])

subplot(6,1,1)
ymax = 1.1*max(max(q));
ymin = 1.1*min(min(q));
axis([t(1,1) t(end,1) ymin ymax])
hold on
grid on
plot(t,q(:,1),'LineWidth',LW)
plot(t,q(:,2),'LineWidth',LW)
plot(t,q(:,3),'LineWidth',LW)
plot(t,q(:,4),'LineWidth',LW)
plot(t,q(:,5),'LineWidth',LW)
plot(t,q(:,6),'LineWidth',LW)
legend('$q_1$','$q_2$','$q_3$','$q_4$','$q_5$','$q_6$','Interpreter','latex')
legend('color','none','Location','eastoutside')
legend('FontSize',font_size)
ylabel('posici\''on [$^\circ$]','Interpreter','latex','FontSize',font_size)
xlabel('tiempo [$s$]','Interpreter','latex','FontSize',font_size)

subplot(6,1,2)
ymax = 1.1*max(max(dq));
ymin = 1.1*min(min(dq));
axis([t(1,1) t(end,1) ymin ymax])
hold on
grid on
plot(t,dq(:,1),'LineWidth',LW)
plot(t,dq(:,2),'LineWidth',LW)
plot(t,dq(:,3),'LineWidth',LW)
plot(t,dq(:,4),'LineWidth',LW)
plot(t,dq(:,5),'LineWidth',LW)
plot(t,dq(:,6),'LineWidth',LW)
legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$','$\dot{q}_4$','$\dot{q}_5$','$\dot{q}_6$','Interpreter','latex')
legend('color','none','Location','eastoutside')
legend('FontSize',font_size)
ylabel('velocidad [$^\circ/s$]','Interpreter','latex','FontSize',font_size)
xlabel('tiempo [$s$]','Interpreter','latex','FontSize',font_size)

subplot(6,1,3)
ymax = 1.1*max(max(ddq));
ymin = 1.1*min(min(ddq));
axis([t(1,1) t(end,1) ymin ymax])
hold on
grid on
plot(t,ddq(:,1),'LineWidth',LW)
plot(t,ddq(:,2),'LineWidth',LW)
plot(t,ddq(:,3),'LineWidth',LW)
plot(t,ddq(:,4),'LineWidth',LW)
plot(t,ddq(:,5),'LineWidth',LW)
plot(t,ddq(:,6),'LineWidth',LW)
legend('$\ddot{q}_1$','$\ddot{q}_2$','$\ddot{q}_3$','$\ddot{q}_4$','$\ddot{q}_5$','$\ddot{q}_6$','Interpreter','latex')
legend('color','none','Location','eastoutside')
legend('FontSize',font_size)
ylabel('acceleraci\''on [$^\circ/s^2$]','Interpreter','latex','FontSize',font_size)
xlabel('tiempo [$s$]','Interpreter','latex','FontSize',font_size)

subplot(6,1,4)
ymax = 1.1*max(max(dddq));
ymin = 1.1*min(min(dddq));
axis([t(1,1) t(end,1) ymin ymax])
hold on
grid on
plot(t,dddq(:,1),'LineWidth',LW)
plot(t,dddq(:,2),'LineWidth',LW)
plot(t,dddq(:,3),'LineWidth',LW)
plot(t,dddq(:,4),'LineWidth',LW)
plot(t,dddq(:,5),'LineWidth',LW)
plot(t,dddq(:,6),'LineWidth',LW)
legend('$j_1$','$j_2$','$j_3$','$j_4$','$j_5$','$j_6$','Interpreter','latex')
legend('color','none','Location','eastoutside')
legend('FontSize',font_size)
ylabel('tir\''on [$^\circ/s^3$]','Interpreter','latex','FontSize',font_size)
xlabel('tiempo [$s$]','Interpreter','latex','FontSize',font_size)

subplot(6,1,5)
ymax = 1.1*max(max(Tz));
ymin = 1.1*min(min(Tz));
axis([t(1,1) t(end,1) ymin ymax])
hold on
grid on
plot(Tz(:,1),Tz(:,2),'LineWidth',LW)
plot(Tz(:,1),Tz(:,3),'LineWidth',LW)
plot(Tz(:,1),Tz(:,4),'LineWidth',LW)
plot(Tz(:,1),Tz(:,5),'LineWidth',LW)
plot(Tz(:,1),Tz(:,6),'LineWidth',LW)
plot(Tz(:,1),Tz(:,7),'LineWidth',LW)
legend('$\tau_1$','$\tau_2$','$\tau_3$','$\tau_4$','$\tau_5$','$\tau_6$','Interpreter','latex')
legend('color','none','Location','eastoutside')
legend('FontSize',font_size)
ylabel('torque $g_z$ [$Nm$]','Interpreter','latex','FontSize',font_size)
xlabel('tiempo [$s$]','Interpreter','latex','FontSize',font_size)

subplot(6,1,6)
ymax = 1.1*max(max(Ty));
ymin = 1.1*min(min(Ty));
axis([t(1,1) t(end,1) ymin ymax])
hold on
grid on
plot(Ty(:,1),Ty(:,2),'LineWidth',LW)
plot(Ty(:,1),Ty(:,3),'LineWidth',LW)
plot(Ty(:,1),Ty(:,4),'LineWidth',LW)
plot(Ty(:,1),Ty(:,5),'LineWidth',LW)
plot(Ty(:,1),Ty(:,6),'LineWidth',LW)
plot(Ty(:,1),Ty(:,7),'LineWidth',LW)
legend('$\tau_1$','$\tau_2$','$\tau_3$','$\tau_4$','$\tau_5$','$\tau_6$','Interpreter','latex')
legend('color','none','Location','eastoutside')
legend('FontSize',font_size)
ylabel('torque $g_y$ [$Nm$]','Interpreter','latex','FontSize',font_size)
xlabel('tiempo [$s$]','Interpreter','latex','FontSize',font_size)

end
