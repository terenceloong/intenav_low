n = size(nav,1);
t = (0:n-1)*dts;

figure
set(gcf,'position',[200,200,350,350])
subplot(3,1,1)
plot(t, error(:,1), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-1,1])
xlabel('时间(s)')
ylabel('北向位置误差(m)')
title('初始对准位置误差曲线')
grid on
subplot(3,1,2)
plot(t, error(:,2), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-1,1])
xlabel('时间(s)')
ylabel('东向位置误差(m)')
grid on
subplot(3,1,3)
plot(t, error(:,3), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-1,1])
xlabel('时间(s)')
ylabel('高度误差(m)')
grid on

figure
set(gcf,'position',[200,200,350,350])
subplot(3,1,1)
plot(t, error(:,4), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-0.4,0.4])
xlabel('时间(s)')
ylabel('北向速度误差(m/s)')
title('初始对准速度误差曲线')
grid on
subplot(3,1,2)
plot(t, error(:,5), 'LineWidth',1.2)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-0.4,0.4])
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
% set(gca, 'ylim', [-6,2])
xlabel('时间(s)')
ylabel('航向角误差(°)')
title('初始对准姿态误差曲线')
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

n = size(bias_esti,1);
t = (1:n)*dts;

figure
set(gcf,'position',[200,200,450,250])
plot(t, drift_g(3:2:end,1)+gyro_bias(1)/pi*180, 'LineWidth',1.2)
hold on
plot(t, bias_esti(:,3), 'LineWidth',1.5)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-0.1,0.1]+gyro_bias(1)/pi*180)
legend('预设值','估计值', 'Location','best')
xlabel('时间(s)')
ylabel('x轴陀螺仪零偏(°/s)')
title('x轴陀螺零仪偏估计曲线')
grid on

figure
set(gcf,'position',[200,200,450,250])
plot(t, drift_g(3:2:end,2)+gyro_bias(2)/pi*180, 'LineWidth',1.2)
hold on
plot(t, bias_esti(:,4), 'LineWidth',1.5)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-0.1,0.1]+gyro_bias(2)/pi*180)
legend('预设值','估计值', 'Location','best')
xlabel('时间(s)')
ylabel('y轴陀螺仪零偏(°/s)')
title('y轴陀螺仪零偏估计曲线')
grid on

figure
set(gcf,'position',[200,200,450,250])
plot(t, drift_g(3:2:end,3)+gyro_bias(3)/pi*180, 'LineWidth',1.2)
hold on
plot(t, bias_esti(:,5), 'LineWidth',1.5)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-0.1,0.1]+gyro_bias(3)/pi*180)
legend('预设值','估计值', 'Location','best')
xlabel('时间(s)')
ylabel('z轴陀螺仪零偏(°/s)')
title('z轴陀螺仪零偏估计曲线')
grid on

figure
set(gcf,'position',[200,200,450,250])
plot(t, acc_bias(3)*ones(1,n), 'LineWidth',1.2)
hold on
plot(t, bias_esti(:,6), 'LineWidth',1.5)
set(gca, 'xlim', [t(1),t(end)])
% set(gca, 'ylim', [-0.1,0.1]+acc_bias(3))
legend('预设值','估计值', 'Location','best')
xlabel('时间(s)')
ylabel('z轴加速度计零偏(°/s)')
title('z轴加速度计零偏估计曲线')
grid on