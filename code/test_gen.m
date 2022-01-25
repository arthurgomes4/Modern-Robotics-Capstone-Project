% phi, x, y, j1, .. , j5, w1, .. ,w4, gs

phi = [ 0:0.01:2*pi ];

phi = [ phi' zeros(length(phi),12) ]

j1 = [ -pi/2:0.05:pi/2 ]';
j1 = [ zeros(length(j1),3),j1, zeros(length(j1),9)];

j5 = [ -pi/2:0.05:pi/2 ]';
j5 = [ zeros(length(j5),7),j5, zeros(length(j5),5)]; 

x = [ 0:0.01:2 ]';
x = [ zeros(length(x),2), x, zeros(length(x), 10)];

w = [ 0:0.02:6*pi]';
w = [ zeros(length(w), 11), w, zeros(length(w),1)];

csvwrite('w4.csv', w);