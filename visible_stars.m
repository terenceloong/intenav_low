%select visible satrs for all

function sv = visible_stars(sv, bate)

m = 1;
while m<size(sv,1)
    if sv(m,10)>bate %altitude angle>bate
        m = m+1;
    else
        sv(m,:) = [];
    end
end

end