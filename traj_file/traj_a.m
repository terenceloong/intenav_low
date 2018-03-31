%accelerate

if t==0
    %-------------- init --------------%
    T = 200;
    n = T/dt*2+1;
	angle = zeros(n,3); %deg
    speed = zeros(n,3); %m/s
    gpsflag = zeros(T/dt+1,1); %0/1
    %---------------------------------------------------------------------%
    p0 = [30, 120, 300]; %deg, [lat,lon,h]
    v0 = [200, 0, 40]; %m/s,deg, [horizontal velocity, down velocity, velocity direction]
    att0 = [40, 2, 0]; %deg, [psi,theta,gamma]
    %---------------------------------------------------------------------%
    Cnb = angle2dcm(att0(1)/180*pi, att0(2)/180*pi, att0(3)/180*pi);
    vh = v0(1);
    vd = v0(2);
    vy = v0(3);
    vn0 = [vh*cosd(vy), vh*sind(vy), vd];
    vb0 = vn0*Cnb';
    angle(1,:) = att0;
    speed(1,:) = vn0;
    maneu_start = [90,120]; %%%%%%
    maneu_end = [102,132]; %%%%%%
else
    angle(k,:) = angle(k-1,:);
    speed(k,:) = speed(k-1,:);
    if mod(k-1,dtgps/dt*2)==0
        gpsflag((k-1)/2+1) = 1;
    end
    
    %-------------- yaw --------------%

    %-------------- pith --------------%
%     if 90<t && t<=92
%         angle(k,2) = 4*(t-90)/2 + att0(2);
%     elseif 98<t && t<=100
%         angle(k,2) = 4*(100-t)/2 + att0(2);
%     elseif 120<t && t<=122
%         angle(k,2) = -4*(t-120)/2 + att0(2);
%     elseif 128<t && t<=130
%         angle(k,2) = -4*(130-t)/2 + att0(2);
%     end
    
    %-------------- roll --------------%

    %-------------- vh --------------%
    if 90<t && t<=100
        vh = 2*(t-90)+200;
    elseif 120<t && t<=130
        vh = 2*(130-t)+200;
    end

    %-------------- vd --------------%
%     if 90<t && t<=100
%         vd = -5*(t-90);
%     elseif 120<t && t<=130
%         vd = -5*(130-t);
%     end

    %-------------- vy --------------%
    
    %-------------- GPS --------------%
    
    %*********************************************************************%
    speed(k,:) = [vh*cosd(vy), vh*sind(vy), vd];
end