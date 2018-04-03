n = size(nav,1);
t = (0:n-1)*dts;

figure
set(gcf,'position',[200,200,350,350])
subplot(3,1,1)
plot(t, error(:,1), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
set(gca, 'ylim', [-1,1])
xlabel('时间(s)')
ylabel('北向位置误差(m)')
title('导航位置误差曲线')
grid on
subplot(3,1,2)
plot(t, error(:,2), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
set(gca, 'ylim', [-1,1])
xlabel('时间(s)')
ylabel('东向位置误差(m)')
grid on
subplot(3,1,3)
plot(t, error(:,3), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
set(gca, 'ylim', [-1,1])
xlabel('时间(s)')
ylabel('高度误差(m)')
grid on

figure
set(gcf,'position',[200,200,350,350])
subplot(3,1,1)
plot(t, error(:,4), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-0.4,0.4])
% set(gca, 'ylim', [-0.5,3])
xlabel('时间(s)')
ylabel('北向速度误差(m/s)')
title('导航速度误差曲线')
grid on
subplot(3,1,2)
plot(t, error(:,5), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-0.4,0.4])
% set(gca, 'ylim', [-1,1])
xlabel('时间(s)')
ylabel('东向速度误差(m/s)')
grid on
subplot(3,1,3)
plot(t, error(:,6), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-0.2,0.2])
xlabel('时间(s)')
ylabel('地向速度误差(m/s)')
grid on

figure
set(gcf,'position',[200,200,350,350])
subplot(3,1,1)
plot(t, error(:,7), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-3,1])
xlabel('时间(s)')
ylabel('航向角误差(°)')
title('导航姿态误差曲线')
grid on
subplot(3,1,2)
plot(t, error(:,8), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-0.5,0.5])
xlabel('时间(s)')
ylabel('俯仰角误差(°)')
grid on
subplot(3,1,3)
plot(t, error(:,9), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-0.5,0.5])
xlabel('时间(s)')
ylabel('滚转角误差(°)')
grid on