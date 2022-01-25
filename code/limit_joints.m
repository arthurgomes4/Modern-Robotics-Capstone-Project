function ijp = limit_joints(ijp, jlim)
% jl 5x2
% ijp 5x1

ijp = ijp(:);

for i = 1:5
    if ijp(i) < jlim(i,1)
        ijp(i) = jlim(i,1);
    elseif ijp(i) > jlim(i,2)
        ijp(i) = jlim(i,2);
    end
end