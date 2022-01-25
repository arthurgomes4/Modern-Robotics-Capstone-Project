function row = se3ToRow(T)
row = [ T(1,1:3), T(2,1:3), T(3,1:3), T(1:3,4)' ]; 
end