%test drift
clear;clc;

T = 200;
dt = 0.01;
n = (T+100)/dt; %100s is used to make filter stable
tr = 40; %correlation time
fw = [10,4];
gain = 0.05;

%--generate random sequence which correlation time is tr--%
data = zeros(n,3);
data(1,1) = randn(1);
for k=2:n
    data(k,1) = data(k-1,1);
    if mod(k-1,tr/dt)==0
        data(k,1) = randn(1);
    end
end

%--two levels filter--%
for p=1:2
    fc = 1/tr/fw(p);
    tao = 1/(2*pi*fc);
    x0 = data(1,p);
    for k=2:n
        x = data(k,p);
        y0 = data(k-1,p+1);
        y = (dt*(x+x0)-(dt-2*tao)*y0)/(dt+2*tao); %one order filter
        x0 = x;
        data(k,p+1) = y;
    end
end

data = data(100/dt+1:end,:);
n = size(data,1);

disp(['max=',num2str(max(data(:,3))*gain)])
disp(['min=',num2str(min(data(:,3))*gain)])

plot((1:n)*dt,data(:,1), 'Color','g')
hold on
plot((1:n)*dt,data(:,2), 'Color','c')
plot((1:n)*dt,data(:,3), 'Color','r', 'LineWidth',1.5)
plot((1:n)*dt,data(:,3)*gain, 'Color','b', 'LineWidth',2)
xlabel('\itt\rm(s)')
legend('origin','first filter','second filter','true','Location','northeast')
grid on
hold off