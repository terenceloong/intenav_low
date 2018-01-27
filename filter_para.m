%Kalman Filter parameter
switch file_name
    case 'intenav_rate_qn_1_test'
        In = eye(14);
        P0 = diag([[1,1,1]*3e-3, [1,1,1]*4, [1,1]*1e-8, 25, [1,1,1]*3e-4, 1, 1e-2]);
        Q = diag([[1,1,1]*gyro_noise/dt, [1,1,1]*acc_noise/dt, [2.5e-16,2.5e-16,1e-2]*1, [1,1,1]*2e-7*1, 1e-2*1, 1e-4*0]);
        R_rou = (sigma3_rou/3)^2;
        R_drou = (sigma3_drou/3)^2;
        R_psi = 1e-4;
    case 'intenav_rate_qn_1_acc'
        In = eye(15);
        P0 = diag([[1,1,1]*3e-3, [1,1,1]*4, [1,1]*1e-8, 25, [1,1,1]*3e-4, 1e-2, 1, 1e-2]);
        Q = diag([[1,1,1]*gyro_noise/dt, [1,1,1]*acc_noise/dt, [2.5e-16,2.5e-16,1e-2]*1, [1,1,1]*2e-7*1, 1e-2, 1e-2*1, 1e-4*0]);
        R_rou = (sigma3_rou/3)^2;
        R_drou = (sigma3_drou/3)^2;
        R_psi = 1e-4;
	case 'intenav_rate_qn_1'
        In = eye(15);
        P0 = diag([[1,1,1]*3e-3, [1,1,1]*4, [1,1]*1e-8, 25, [1,1,1]*3e-4, 1e-2, 1, 1e-2]);
        Q = diag([[1,1,1]*gyro_noise/dt, [1,1,1]*acc_noise/dt, [2.5e-16,2.5e-16,1e-2]*1, [1,1,1]*2e-7*1, 1e-2, 1e-2*1, 1e-4*0]);
        R_rou = (sigma3_rou/3)^2;
        R_drou = (sigma3_drou/3)^2;
        R_psi = 1e-4;
end