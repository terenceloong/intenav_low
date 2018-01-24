%Kalman Filter parameter

In = eye(14);
P0 = diag([[1,1]*3e-3, 3e-3, [1,1,1]*4, [1,1]*1e-8, 25, [1,1]*3e-4, 3e-4, 1, 1e-2]);
Q = diag([[1,1,1]*gyro_noise/dt, [1,1,1]*acc_noise/dt, [2.5e-16,2.5e-16,1e-2]*1, [1,1,1]*3e-7*1, 1e-2*1, 1e-4*0]);
R_rou = (sigma3_rou/3)^2;
R_drou = (sigma3_drou/3)^2;
R_psi = 1e-4;