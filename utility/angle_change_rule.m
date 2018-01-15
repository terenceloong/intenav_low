%show the change rule of attitude angle when the vehicle ratates along acceleration vector

a = 1; %horizontal acceleration
att = [0,0,0]; %deg, the attitude of vehicle
dpsi = 0; %deg, the difference between velocity direction and heading

fn = [a*cosd(att(1)+dpsi);a*sind(att(1)+dpsi);-1];
fn = fn/norm(fn);
att = att/180*pi;
Cnb = angle2dcm(att(1),att(2),att(3));
fb = Cnb*fn;

alpha=0:0.05:1; %deg
n = length(alpha);
angle = zeros(n,3);

for k=1:n
    q = [cosd(alpha(k)/2),fb'*sind(alpha(k)/2)]; %the quaternion from b to p
    Cbp = quat2dcm(q);
    Cnp = Cbp*Cnb;
    [r1,r2,r3] = dcm2angle(Cnp);
    angle(k,:) = ([r1,r2,r3]-att) /pi*180;
end

plot(alpha,angle(:,1), 'LineWidth',1)
hold on
plot(alpha,angle(:,2), 'LineWidth',1)
plot(alpha,angle(:,3), 'LineWidth',1)
xlabel('\alpha(\circ)')
ylabel('(\circ)')
legend('\delta\psi','\delta\theta','\delta\gamma','Location','northwest')
grid on
hold off

clearvars