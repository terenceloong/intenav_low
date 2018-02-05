function data = sv_ecef(almanac, time, p, v, error)
%calculate the coordinate of sv at ecef. (x,y,z)
%almanac format:[ID, a, e, i, Omega0, omega, M0, toe, OmegaDot]
%time = [gps_week, second], dividing two parts to avoid a very big number
%p = [latitude, longitude, altitude] (row/column), deg
%v = [vn, ve, vd] (row/column), m/s
%error = [noise_rou, noise_drou, dtr, dtv], 3sigma
%data = [ID, x,y,z, vx,vy,vz, pseudo-range, pseudo-range rate, altitude-angle,direction-angle, xun,yun,zun]
%altitude-angle,direction-angle:deg
%(xun,yun,zun) is the unit vector from vehicle to sv at ned

sigma_rou = error(1)/3; %m, 1sigma
sigma_drou = error(2)/3; %m/s, 1sigma
dtr = error(3); %the distance of clock error
dtv = error(4); %the velocity of clock error rate

num = size(almanac,1);
data = zeros(num,10);
data(:,1) = almanac(:,1); %ID
toe = almanac(1,8);
t = (time(1)-almanac(1,12))*604800 + (time(2)-toe); %s
miu = 3.986005e14;
w = 7.2921151467e-5;

Cen = dcmecef2ned(p(1), p(2));
rp = lla2ecef([p(1), p(2), p(3)])'; %(ecef)
vp = Cen'*[v(1); v(2); v(3)]; %(ecef)

for k=1:num
    a = almanac(k,2);
    n = sqrt(miu/a^3); %mean motion
    M = mod(almanac(k,7)+n*t, 2*pi); %0-2*pi
    e = almanac(k,3);
    E = kepler(M, e); %0-2*pi
    f = 2*mod(atan(sqrt((1+e)/(1-e))*tan(E/2)), pi); %0-2*pi
    phi = f+almanac(k,6);
    i = almanac(k,4);
    Omega = almanac(k,5) + (almanac(k,9)-w)*t - w*toe;
    
    r = a*(1-e*cos(E));
    x = r*cos(phi);
    y = r*sin(phi);
    rs = [x*cos(Omega)-y*cos(i)*sin(Omega); x*sin(Omega)+y*cos(i)*cos(Omega); y*sin(i)]; %(ecef)
    data(k,2:4) = rs'; %x,y,z
    
    rps = rs-rp; %the vector from p to sv (ecef)
    rou = norm(rps);
    data(k,8) = rou + dtr + randn(1)*sigma_rou; %pseudo range, m
    
    rpsu = rps/rou; %unit vector (ecef)
    rpsu_n = Cen*rpsu; %(ned)
    data(k,12:14) = rpsu_n'; %xun,yun,zun
    data(k,10) = asind(-rpsu_n(3)); %altitude angle, deg
    data(k,11) = atan2d(rpsu_n(2),rpsu_n(1)); %direction angle, deg
    
    h = sqrt(miu*a*(1-e^2));
    vr = miu/h*e*sin(f);
    vt = miu/h*(1+e*cos(f));
    vx = vr*cos(phi) - vt*sin(phi);
    vy = vr*sin(phi) + vt*cos(phi);
    vs = [vx*cos(Omega)-vy*cos(i)*sin(Omega); vx*sin(Omega)+vy*cos(i)*cos(Omega); vy*sin(i)] - cross([0;0;w],rs); %(ecef)
    data(k,5:7) = vs'; %vx,vy,vz
    data(k,9) = (vs-vp)'*rpsu + dtv + randn(1)*sigma_drou; %pseudo range rate, m/s
end

end

function E = kepler(M, e)
    E = M;
    Ei = E - (E-e*sin(E)-M)/(1-e*cos(E));
    while abs(Ei-E) > 1e-10
        E = Ei;
        Ei = E - (E-e*sin(E)-M)/(1-e*cos(E));
    end
end