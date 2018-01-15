%loacte and measure velocity with 4 GPS stars

%sv = [ID, x,y,z, vx,vy,vz, pseudo-range, pseudo-range rate, altitude-angle]
%nav = [lat, lon, h, dtr, vn, ve, vd, dtv], deg
function nav = gps_4stars(sv)

R = [sv(1,8);sv(2,8);sv(3,8);sv(4,8)]; %pseudo range
V = [sv(1,9);sv(2,9);sv(3,9);sv(4,9)]; %pseudo range rate
x0 = [0;0;0;0];
cnt = 0;
while 1
    r1 = sv(1,2:4)-x0(1:3)';
    r2 = sv(2,2:4)-x0(1:3)';
    r3 = sv(3,2:4)-x0(1:3)';
    r4 = sv(4,2:4)-x0(1:3)';
    e1= r1/norm(r1);
    e2= r2/norm(r2);
    e3= r3/norm(r3);
    e4= r4/norm(r4);
    G = [e1,-1;e2,-1;e3,-1;e4,-1];
    S = [e1*sv(1,2:4)';e2*sv(2,2:4)';e3*sv(3,2:4)';e4*sv(4,2:4)'];
    x = G\(S-R);
    cnt = cnt+1;
    if cnt == 10
        break
    end
    if norm(x-x0)<0.01
        break
    end
    x0 = x;
end
r1 = sv(1,2:4)-x(1:3)';
r2 = sv(2,2:4)-x(1:3)';
r3 = sv(3,2:4)-x(1:3)';
r4 = sv(4,2:4)-x(1:3)';
e1= r1/norm(r1);
e2= r2/norm(r2);
e3= r3/norm(r3);
e4= r4/norm(r4);
G = [e1,-1;e2,-1;e3,-1;e4,-1];
U = [e1*sv(1,5:7)';e2*sv(2,5:7)';e3*sv(3,5:7)';e4*sv(4,5:7)'];
v = G\(U-V);
p = ecef2lla(x(1:3)', 'WGS84');
Cen = dcmecef2ned(p(1), p(2));
v(1:3) = Cen*v(1:3);
nav = [p,x(4),v'];
    
end