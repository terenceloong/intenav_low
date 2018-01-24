%deep-coupled integrated navigation, navigation solve in a step.
%navigation frame is NED, IMU's output is rate.
time1 = clock;
earth_constant;

%--almanac--%
[gps_almanac, gps_week, second] = load_almanac('949_405504.txt', '11/04/2017', '10:10:20');

%--navigation initial value--%
p = traj(1,1:3)' + [[3; 3]/a/pi*180; 5]; %deg
v = traj(1,4:6)' + [1; 1.2; -1.5]; %m/s
att = traj(1,7:9)' + [2.7; 2.3; 1.3]; %deg
pva0 = [p;v;att]'; %record initial value

%--step--%
sample = 2;
dts = dt*sample;
n = floor((size(imu,1)-1)/sample); %the number of inertial solving

%--GPS character--%
sigma3_rou = 5; %m 3sigma
sigma3_drou = 0.2; %m 3sigma
dtr = 1*1; %m
dtv = 0.1*1; %m/s

%--filter--%
filter_para;

%*************************************************************************%
%--store--%
nav = zeros(n,9); %[lat, lon, h, vn, ve, vd, yaw, pitch, roll]
filter = zeros(n,5); %[ex,ey,ez, dtr,dtv]
error_gps = zeros(n,6);
output_Xc = zeros(n,14); %%%
output_P = zeros(n,14); %%%
output = zeros(n,14); %%%

%--initialize--%
avp = nav_init(p, v, att);
P = P0;
s = In;
X = zeros(14,1); %[dpsix,dpsiy,dpsiz, dvn,dve,dvd, dlat,dlon,dh, ex,ey,ez, dtr, dtv]
dgyro = [0;0;0]; %gyro compensation
dacc = [0;0;0]; %accelerometer compensation
psi0 = att(1)/180*pi; %rad, reference yaw angle
sv4_num = [100,100,100,100]; %the number of selected satellites
Xci = zeros(14,10);

for k=1:n
    t = k*dts;
    
    %--IMU data--%
    kj = sample*k+1;
    if sample==1
        gyro0 = (imu(kj-1, 1:3)'-dgyro) /180*pi;
        gyro2 = (imu(kj  , 1:3)'-dgyro) /180*pi;
        gyro1 = (gyro0+gyro2)/2;
        acc0  = (imu(kj-1, 4:6)'-dacc);
        acc2  = (imu(kj  , 4:6)'-dacc);
        acc1  = (acc0+acc2)/2;
    else
        gyro0 = (imu(kj-2, 1:3)'-dgyro) /180*pi;
        gyro1 = (imu(kj-1, 1:3)'-dgyro) /180*pi;
        gyro2 = (imu(kj  , 1:3)'-dgyro) /180*pi;
        acc0  = (imu(kj-2, 4:6)'-dacc);
        acc1  = (imu(kj-1, 4:6)'-dacc);
        acc2  = (imu(kj  , 4:6)'-dacc);
    end
    
    %--GPS output--%
    error_gps(k,:) = [0,0,0,0,0,0];
    if gpsflag(kj)==1
        sv = sv_ecef(gps_almanac, [gps_week,second+t], traj(kj,1:3), traj(kj,4:6), [sigma3_rou,sigma3_drou,dtr+dtv*t,dtv]);
        sv = visible_stars(sv, 10);
        sv4_row = [find(sv(:,1)==sv4_num(1)),find(sv(:,1)==sv4_num(2)),find(sv(:,1)==sv4_num(3)),find(sv(:,1)==sv4_num(4))];
        if length(sv4_row)<4 || mod(t,30)==0 %select stars every 30s
            sv4_num = select_4stars(sv);
            sv4_row = [find(sv(:,1)==sv4_num(1)),find(sv(:,1)==sv4_num(2)),find(sv(:,1)==sv4_num(3)),find(sv(:,1)==sv4_num(4))];
        end
        sv4 = sv(sv4_row,:);
        gps = gps_4stars(sv4);
        error_gps(k,1:2) = (gps(1:2)-traj(kj,1:2))/180*pi*a;
        error_gps(k,3) = gps(3)-traj(kj,3);
        error_gps(k,4:6) = gps(5:7)-traj(kj,4:6);
        sv = sv4;
    end
    
    %--inertial navigation solution--%
    K1 = fun_dx_qn(avp, [gyro0;acc0]);
    K2 = fun_dx_qn(avp+dts*K1/2, [gyro1;acc1]);
    K3 = fun_dx_qn(avp+dts*K2/2, [gyro1;acc1]);
    K4 = fun_dx_qn(avp+dts*K3, [gyro2;acc2]);
    avp = avp + (K1+2*K2+2*K3+K4)*dts/6;
    avp(1:4) = quatnormalize(avp(1:4)')'; %quaternion normalization
    
%=========================================================================%
    %----------time update----------%
    [Phi, Gamma] = state_matrix(avp, acc1, dts);
    X = Phi*X;
    P = s*P*s;
    P = Phi*P*Phi' + Gamma*Q*Gamma';

    if gpsflag(kj)==1
        %----------claculate measure matrix----------%
        [H, Z, ng] = measure_matrix(avp, sv);
        R = diag([ones(1,ng)*R_rou, ones(1,ng)*R_drou]);
        
%         if t<50
%         if acc2(2)<1
            H = [H; zeros(1,14)];
            H(end,3) = -1;
            R = [R, zeros(2*ng,1)];
            R = [R; zeros(1,2*ng+1)];
            R(end,end) = R_psi;
%             psi0 = atan2(gps(6),gps(5));
            psi0 = traj(kj,7)/180*pi;
            [psi,~,~] = dcm2angle(quat2dcm(avp(1:4)'));
            dpsi = psi-psi0;
            if t>50
                dpsi = dpsi+0.1/180*pi;
            end
            if dpsi>pi
                dpsi = dpsi-2*pi;
            elseif dpsi<-pi
                dpsi = dpsi+2*pi;
            end
            Z = [Z; dpsi+randn(1)*0.01];
%         end

        %----------measure update----------%
        K = P*H' / (H*P*H'+R);
        r = Z - H*X;
        Xc = K*r;
        X = X + Xc;
        P = (In-K*H)*P*(In-K*H)' + K*R*K';
        output(k,1:8) = r(1:8)'; %%%
        output_Xc(k,:) = Xc'; %%%
        
        %---------fading----------%
%         s = In*sqrt(1.002);
%         s(10,10) = 1;
%         if t>20
%             Xci = [Xci(:,2:end),Xc];
%             Xcs = sum(Xci,2);
%             Xcp = abs(Xcs./sqrt(diag(P)));
%             Xcp = 1+0.008*(1-exp(-Xcp*1.2));
%             s = diag(sqrt(Xcp));
%             s(10,10) = 1;
%         end
        
        %---------adjust----------%
        if norm(X(1:3))>0
            phi = norm(X(1:3));
            qc = [cos(phi/2), X(1:3)'/phi*sin(phi/2)];
            avp(1:4) = quatmultiply(qc, avp(1:4)')';
        end
        avp(5:10) = avp(5:10) - X(4:9);
        X(1:9) = zeros(9,1);
    end
    
    %---------gyro compensation----------%
%     if t==20
%         dgyro = X(10:12)/pi*180;
%         X(10:12) = [0;0;0];
%     end

    %---------store filter----------%
    filter(k,1:3) = X(10:12)'/pi*180;
    filter(k,4:5) = X(13:14)';
%     filter(k,6:8) = X(1:3)'/pi*180;
%     filter(k,9:14) = X(4:9);
    output_P(k,:) = sqrt(diag(P))'; %%%
%=========================================================================%
    
    %--store--%
    nav(k,1:2) = avp(8:9)' /pi*180; %deg
    nav(k,3) = avp(10); %m
    nav(k,4:6) = avp(5:7)'; %m/s
    [r1,r2,r3] = quat2angle(avp(1:4)');
    nav(k,7:9) = [r1,r2,r3] /pi*180; %deg
    output(k,1:3) = acc1'*angle2dcm(r1+5.5/180*pi,r2+0.0/180*pi,r3-1.1/180*pi);
end
nav = [pva0; nav];

time2 = clock;
disp(['Integrated navigation completes: ',num2str(mod(time2(5)+60-time1(5),60)*60+time2(6)-time1(6)),'s'])
plot_naverror;

%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++%
function [gps_almanac, gps_week, second] = load_almanac(file, date, time)
	gps_almanac = read_gps_almanac(['.\gps_almanac\',file]);
    day = datenum(date,'mm/dd/yyyy')-730354; %730354: 08/22/1999, No.0 week
    gps_week = floor(day/7);
    second = (day-gps_week*7)*24*3600 + clocksec(time);
end

function avp = nav_init(p, v, att)
    p(1:2) = p(1:2)/180*pi;
    att = att/180*pi;
    q = angle2quat(att(1), att(2), att(3));
    avp = [q'; v; p];
    %avp = [q1; q2; q3; q4; vn; ve; vd; lat; lon; h];
    %                           m/s       rad     m
end

function [Phi, Gamma] = state_matrix(avp, fb, dts)
    global a f w
    v = avp(5:7);
    lat = avp(8);
    h = avp(10);
    Rm = (1-f)^2*a / (1-(2-f)*f*sin(lat)^2)^1.5;
    Rn =         a / (1-(2-f)*f*sin(lat)^2)^0.5;
    wien = [w*cos(lat); 0; -w*sin(lat)];
    wenn = [v(2)/(Rn+h); -v(1)/(Rm+h); -v(2)/(Rn+h)*tan(lat)];
    winn = antisym(wien+wenn);
    w2inn = antisym(2*wien+wenn);
    Cbn = quat2dcm(avp(1:4)')';
    fn = antisym(Cbn*fb);
    E1 = [     0,        1/(Rn+h),    0;
          -1/(Rm+h),         0,       0;
               0,   -tan(lat)/(Rn+h), 0];
    E2 = diag([1/(Rm+h), sec(lat)/(Rn+h), -1]);
    A = zeros(14);
%     A(1:3,1:3) = -winn; %
%     A(1:3,4:6) = E1; %
    A(1:3,10:12) = -Cbn;
    A(4:6,1:3) = fn;
%     A(4:6,4:6) = -w2inn; %
    A(7:9,4:6) = E2;
    A(13,14) = 1;
    Phi = eye(14)+A*dts+(A*dts)^2/2; %--Phi
    Gamma = eye(14);
    Gamma(1:3,1:3) = -Cbn;
    Gamma(4:6,4:6) = Cbn;
    Gamma = Gamma*dts; %--Gmamma
end

function [H, Z, ng] = measure_matrix(avp, sv)
    global a f
    lat = avp(8);
    lon = avp(9);
    h = avp(10);
    Cen = [-sin(lat)*cos(lon), -sin(lat)*sin(lon),  cos(lat);
                    -sin(lon),           cos(lon),         0;
           -cos(lat)*cos(lon), -cos(lat)*sin(lon), -sin(lat)];
    ng = size(sv,1); %ng is the number of selected stars
    rpe = lla2ecef([lat/pi*180, lon/pi*180, h]); %row vector
    rpe = repmat(rpe, ng, 1); %ng row vectors
    vpe = avp(5:7)'*Cen; %row vector
    vpe = repmat(vpe, ng, 1); %ng row vectors
    rse = sv(:,2:4); %ng row vectors
    vse = sv(:,5:7); %ng row vectors
    rps = rse - rpe;
    rou = sum(rps.*rps,2).^0.5; %rou, column vector
    rpsu = rps./(rou*[1,1,1]);
    drou = sum(rpsu.*(vse-vpe),2); %drou, column vector
    He = -rpsu;
    F = [-(a+h)*sin(lat)*cos(lon), -(a+h)*cos(lat)*sin(lon), cos(lat)*cos(lon);
         -(a+h)*sin(lat)*sin(lon),  (a+h)*cos(lat)*cos(lon), cos(lat)*sin(lon);
           (a*(1-f)^2+h)*cos(lat),             0,            sin(lat)        ];
    Ha = He*F;
    Hb = He*Cen';
    H = zeros(2*ng,14);
    H(1:ng,7:9) = Ha;
    H(1:ng,13) = -ones(ng,1);
    H(ng+1:2*ng,4:6) = Hb;
    H(ng+1:2*ng,14) = -ones(ng,1);
    Z = [rou-sv(:,8); drou-sv(:,9)];
end