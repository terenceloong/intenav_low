function alpha = angle_pmpi(alpha)
%transform angle to plus or minus pi.

if alpha>pi
    alpha = alpha-2*pi;
elseif alpha<-pi
    alpha = alpha+2*pi;
end

end