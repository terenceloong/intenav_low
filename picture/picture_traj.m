n = size(traj,1);
t = (0:n-1)*dt;

figure
plot(traj(:,2),traj(:,1), 'LineWidth',2.5)
set(gcf,'position',[200,200,450,300])
xlabel('经度(°)')
ylabel('纬度(°)')
title('机动轨迹位置曲线')
grid on

figure
set(gcf,'position',[200,200,350,350])
subplot(3,1,1)
plot(t, traj(:,4), 'LineWidth',1.5)
set(gca, 'xlim', [t(1),t(end)])
set(gca, 'ylim', [125,160])
xlabel('时间(s)')
ylabel('北向速度(m/s)')
title('机动轨迹速度曲线')
grid on
subplot(3,1,2)
plot(t, traj(:,5), 'LineWidth',1.5)
set(gca, 'xlim', [t(1),t(end)])
set(gca, 'ylim', [100,135])
xlabel('时间(s)')
ylabel('东向速度(m/s)')
grid on
subplot(3,1,3)
plot(t, traj(:,6), 'LineWidth',1.5)
set(gca, 'xlim', [t(1),t(end)])
set(gca, 'ylim', [-25,25])
xlabel('时间(s)')
ylabel('地向速度(m/s)')
grid on

figure
set(gcf,'position',[200,200,350,350])
subplot(3,1,1)
plot(t, traj(:,7), 'LineWidth',1.5)
set(gca, 'xlim', [t(1),t(end)])
xlabel('时间(s)')
ylabel('航向角(°)')
title('机动轨迹姿态曲线')
grid on
subplot(3,1,2)
plot(t, traj(:,8), 'LineWidth',1.5)
set(gca, 'xlim', [t(1),t(end)])
set(gca, 'ylim', [-3,7])
xlabel('时间(s)')
ylabel('俯仰角(°)')
grid on
subplot(3,1,3)
plot(t, traj(:,9), 'LineWidth',1.5)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-12,12])
xlabel('时间(s)')
ylabel('滚转角(°)')
grid on