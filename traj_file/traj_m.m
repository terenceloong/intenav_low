%maneuver

if t==0
    %-------------- init --------------%
    T = 500;
    n = T/dt*2+1;
	angle = zeros(n,3); %deg
    speed = zeros(n,3); %m/s
    gpsflag = zeros(T/dt+1,1); %0/1
    %---------------------------------------------------------------------%
    p0 = [30, 120, 300]; %deg, [lat,lon,h]
    v0 = [200, 0, 90]; %m/s,deg, [horizontal velocity, down velocity, velocity direction]
    att0 = [90, 2, 0]; %deg, [psi,theta,gamma]
    %---------------------------------------------------------------------%
    Cnb = angle2dcm(att0(1)/180*pi, att0(2)/180*pi, att0(3)/180*pi);
    vh = v0(1);
    vd = v0(2);
    vy = v0(3);
    vn0 = [vh*cosd(vy), vh*sind(vy), vd];
    vb0 = vn0*Cnb';
    angle(1,:) = att0;
    speed(1,:) = vn0;
    maneu_start = [60,90,120,150,180,210]; %%%%%%
    maneu_end = [72,102,132,162,192,222]; %%%%%%
else
    angle(k,:) = angle(k-1,:);
    speed(k,:) = speed(k-1,:);
    if mod(k-1,dtgps/dt*2)==0
        gpsflag((k-1)/2+1) = 1;
    end
    
    %-------------- yaw --------------%
    if 60<t && t<=70
        angle(k,1) = 90*(70-t)/10;
    elseif 90<t && t<=100
        angle(k,1) = -90*(t-90)/10;
    elseif 120<t && t<=130
        angle(k,1) = -90*(130-t)/10;
    elseif 150<t && t<=160
        angle(k,1) = 90*(t-150)/10;
    elseif 180<t && t<=190
        angle(k,1) = 90*(190-t)/10;
    elseif 210<t && t<=220
        angle(k,1) = -90*(t-210)/10;
    end

    %-------------- pith --------------%
    if 60<t && t<=62
        angle(k,2) = 2*(t-60)/2 + att0(2);
    elseif 68<t && t<=70
        angle(k,2) = 2*(70-t)/2 + att0(2);
    elseif 90<t && t<=92
        angle(k,2) = 2*(t-90)/2 + att0(2);
    elseif 98<t && t<=100
        angle(k,2) = 2*(100-t)/2 + att0(2);
    elseif 120<t && t<=122
        angle(k,2) = 2*(t-120)/2 + att0(2);
    elseif 128<t && t<=130
        angle(k,2) = 2*(130-t)/2 + att0(2);
    elseif 150<t && t<=152
        angle(k,2) = 2*(t-150)/2 + att0(2);
    elseif 158<t && t<=160
        angle(k,2) = 2*(160-t)/2 + att0(2);
    elseif 180<t && t<=182
        angle(k,2) = 2*(t-180)/2 + att0(2);
    elseif 188<t && t<=190
        angle(k,2) = 2*(190-t)/2 + att0(2);
    elseif 210<t && t<=212
        angle(k,2) = 2*(t-210)/2 + att0(2);
    elseif 218<t && t<=220
        angle(k,2) = 2*(220-t)/2 + att0(2);
    end

    %-------------- roll --------------%
    if 60<t && t<=62
        angle(k,3) = -10*(t-60)/2;
    elseif 68<t && t<=70
        angle(k,3) = -10*(70-t)/2;
    elseif 90<t && t<=92
        angle(k,3) = -10*(t-90)/2;
    elseif 98<t && t<=100
        angle(k,3) = -10*(100-t)/2;
    elseif 120<t && t<=122
        angle(k,3) = 10*(t-120)/2;
    elseif 128<t && t<=130
        angle(k,3) = 10*(130-t)/2;
    elseif 150<t && t<=152
        angle(k,3) = 10*(t-150)/2;
    elseif 158<t && t<=160
        angle(k,3) = 10*(160-t)/2;
    elseif 180<t && t<=182
        angle(k,3) = -10*(t-180)/2;
    elseif 188<t && t<=190
        angle(k,3) = -10*(190-t)/2;
    elseif 210<t && t<=212
        angle(k,3) = -10*(t-210)/2;
    elseif 218<t && t<=220
        angle(k,3) = -10*(220-t)/2;
    end

    %-------------- vh --------------%

    %-------------- vd --------------%

    %-------------- vy --------------%
    if 60<t && t<=70
        vy = 90*(70-t)/10;
    elseif 90<t && t<=100
        vy = -90*(t-90)/10;
    elseif 120<t && t<=130
        vy = -90*(130-t)/10;
    elseif 150<t && t<=160
        vy = 90*(t-150)/10;
    elseif 180<t && t<=190
        vy = 90*(190-t)/10;
    elseif 210<t && t<=220
        vy = -90*(t-210)/10;
    end
    
    %-------------- GPS --------------%
    
    %*********************************************************************%
    speed(k,:) = [vh*cosd(vy), vh*sind(vy), vd];
end