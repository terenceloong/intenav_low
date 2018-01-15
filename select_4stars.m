function number = select_4stars(sv)

G = zeros(4);
G(:,4) = [-1;-1;-1;-1];
top = sv(:,10)==max(sv(:,10)); %altitude angle is maximal
G(1,1:3) = sv(top,12:14);
number = [sv(top,1),0,0,0];
sv(top,:) = [];
n = size(sv,1);
GDOP_min = 100;
for k1=1:n
    G(2,1:3) = sv(k1,12:14);
    for k2=(k1+1):n
        G(3,1:3) = sv(k2,12:14);
        for k3=(k2+1):n
            G(4,1:3) = sv(k3,12:14);
            GDOP = trace(inv(G'*G));
            if GDOP<GDOP_min
                number(2:4) = [sv(k1,1),sv(k2,1),sv(k3,1)];
                GDOP_min = GDOP;
            end
        end
    end
end

end