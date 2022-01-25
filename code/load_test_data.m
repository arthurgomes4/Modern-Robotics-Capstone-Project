% loads the data for function compute_motion()

Tsci = [ 1, 0, 0, 1; 
         0, 1, 0, 0;
         0, 0, 1, 0.025;
         0, 0, 0, 1]
     
% Tscf = [ 0, 1, 0, 0;
%          -1, 0, 0,-1;
%          0, 0, 1, 0.025;
%          0, 0, 0, 1]

% new task
Tscf = [ 1, 0, 0, 1;
         0, 1, 0,-1;
         0, 0, 1, 0.025;
         0, 0, 0, 1]

% with deviation
current_robot_config = [ 0.5238,-0.3,0, 0,0,0,0,0, 0,0,0,0, 0 ]

% without deviation
% current_robot_config = [ 0,0,0, 0,0,0,0,0, 0,0,0,0, 0]

Tse = [ 0, 0, 1, 0;
        0, 1, 0, 0;
       -1, 0, 0, 0.5;
        0, 0, 0, 1 ]
    
% to test feedforward
% Tse = [ 1.0000         0         0    0.1992;
%              0    1.0000         0         0;
%              0         0    1.0000    0.7535;
%              0         0         0    1.0000; ]

Kp = eye(6).*[ 1 1 1 1 1 1 ]
Ki = zeros(6)


