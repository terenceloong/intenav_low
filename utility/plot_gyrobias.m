n = size(filter,1);
t = (1:n)*dts;

figure
plot(t, walk_g(3:2:end,1)+gyro_bias(1)/pi*180)
hold on
plot(t, filter(:,1))
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\epsilon\it_x\rm(\circ)')
grid on

figure
plot(t, walk_g(3:2:end,2)+gyro_bias(2)/pi*180)
hold on
plot(t, filter(:,2))
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\epsilon\it_y\rm(\circ)')
grid on

figure
plot(t, walk_g(3:2:end,3)+gyro_bias(3)/pi*180)
hold on
plot(t, filter(:,3))
set(gca, 'xlim', [t(1),t(end)])
xlabel('\itt\rm(s)')
ylabel('\epsilon\it_z\rm(\circ)')
grid on