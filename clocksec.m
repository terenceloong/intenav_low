%transform time string to second

function sec = clocksec(t)

c = sscanf(t,'%d:%d:%d');
sec = c(1)*3600 + c(2)*60 + c(3);
    
end