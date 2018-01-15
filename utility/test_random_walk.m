%test random walk, find the coefficient of wanted slowly varying curve

dt = 0.01;
n = 100000;
sigma = 0.2;
tr = 100;
fc = 0.001;
tao = 1/(2*pi*fc);

data = zeros(n,3);

x0 = 0;
y0 = 0;
x = 0;
for k=1:n
    w = randn(1)*sigma;
    x = x + (-x/tr+w)*dt;
    y = (dt*(x+x0) - (dt-2*tao)*y0) / (dt+2*tao); %one order filter
	x0 = x;
    y0 = y;
    data(k,:) = [w,x,y];
end

plot((1:n)*dt,data(:,2), 'LineWidth',1)
hold on
plot((1:n)*dt,data(:,3), 'LineWidth',1)
xlabel('\itt\rm(s)')
ylabel('(\circ)')
legend('before filtering','random walk','Location','northeast')
hold off

clearvars