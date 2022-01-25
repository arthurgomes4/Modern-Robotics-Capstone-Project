function t = row2se3(row)
t = [ row(1:3), row(10); row(4:6), row(11); row(7:9), row(12); 0 0 0 1 ];
end