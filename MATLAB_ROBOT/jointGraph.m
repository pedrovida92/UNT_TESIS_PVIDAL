function jointGraph(M)

% Grafica de q(t),dq(t),ddq(t)
h = (M(end,1)-M(1,1))/length(M(:,1));

t = M(:,1);
q = rad2deg(M(:,2:7));
dq = diff(q)/h;
ddq = diff(dq)/h;
figure('Name','Joint Positions','Position',[100 100 1068 600])
ylim = 1.1*max(max(abs(q)));
for i = 1:6
    subplot(2,3,i)
    plot(t,q(:,i),'LineWidth',1,'Color',[0 0.4470 0.7410])
    xlabel('time [s]')
    ylabel(['$' 'q_' num2str(i) '[^\circ]' '$'],'Interpreter','latex')
    axis([t(1,1) t(end,1) -ylim ylim])
    grid on
end

figure('Name','Joint Velocities','Position',[100 100 1068 600])
ylim = 1.1*max(max(abs(dq)));
for i = 1:6
    subplot(2,3,i)
    plot(t(1:length(dq),1),dq(:,i),'LineWidth',1,'Color',[0.4660 0.6740 0.1880])
    xlabel('time [s]')
    ylabel(['$' '\dot{q}_' num2str(i) '[^\circ/s]' '$'],'Interpreter','latex')
    axis([t(1,1) t(end,1) -ylim ylim])
    grid on
end

figure('Name','Joint Accelerations','Position',[100 100 1068 600])
ylim = 1.1*max(max(abs(ddq)));
for i = 1:6
    subplot(2,3,i)
    plot(t(1:length(ddq),1),ddq(:,i),'LineWidth',1,'Color',[0.8500 0.3250 0.0980])
    xlabel('time [s]')
    ylabel(['$' '\ddot{q}_' num2str(i) '[^\circ/s^2]' '$'],'Interpreter','latex')
    axis([t(1,1) t(end,1) -ylim ylim])
    grid on
end

end


