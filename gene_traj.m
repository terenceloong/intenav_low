%generate trajectory
clear;clc;

trajfile = 'traj_u';
dt = 0.01;
dtgps = 0.1;

%--low accurate--%
acc_bias = [2, 3, 1]*0.01 *0; %[mg]
gyro_bias = [0.2, 0.15, -0.1]/180*pi *0; %[deg]
acc_noise = ( 3 /3*0.01)^2*dt *1; %[mg],3sigma
gyro_noise = ( 0.1 /3/180*pi)^2*dt *1; %[deg/s],3sigma
gyro_walk = 0.2 *1; %random walk gain

%--high accurate--%
% acc_bias = [2, 3, 1]*0.01 *0; %[mg]
% gyro_bias = [0.2*0, 0.15*0, -0.1]/180*pi *1; %[deg]
% acc_noise = ( 0.03 /3*0.01)^2*dt *1; %[mg],3sigma
% gyro_noise = ( 0.1/3600 /3/180*pi)^2*dt *1; %[deg/s],3sigma
% gyro_walk = 0.2 *0; %random walk gain

acc_quantize = 0.01 *0; %m/s^2
gyro_quantize = 0.01 *0; %deg/s
noise_seeds = [23093 23094 23095 23096 23097 23098];
% noise_seeds = randi(1e6,1,6);
%***************************************%

k = 0;
while 1
    t = k*dt/2;
    k = k+1;
    eval(trajfile);
    if k==n
        break
    end
end

cmd.time = (0:dt/2:T)';
cmd.signals.values = [angle,speed];
cmd.signals.dimensions = 6;
sim trajectory

%--gyro random walk--%
n = T/dt+1;
tr = 100;
fc = 0.001;
tao = 1/(2*pi*fc);
walk_gx = [0,0,0];
walk_gx0 = [0,0,0];
walk_gy0 = [0,0,0];
walk_g = zeros(n,3);
for k=1:n
    walk_gx = walk_gx + (-walk_gx/tr+randn(1,3)*gyro_walk)*dt;
    walk_gy = (dt*(walk_gx+walk_gx0) - (dt-2*tao)*walk_gy0) / (dt+2*tao);
    walk_gx0 = walk_gx;
    walk_gy0 = walk_gy;
    walk_g(k,:) = walk_gy; %deg
    imu(k,1:3) = imu(k,1:3)+walk_gy;
end