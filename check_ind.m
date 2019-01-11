function val = check_ind(ix,iy,maxind)
val = 1;
if ix<1
    val = 0;
end
if ix>maxind+1
    val = 0;
end

if iy<1
    val = 0;
end
if iy>maxind+1
    val = 0;
end