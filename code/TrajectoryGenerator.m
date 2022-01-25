function traj = TrajectoryGenerator(tse, tsci, tscf, tceg, tces, k)

% SET time parameters for 8 segments 
time = [ 10;  % move to grasp standoff 5
         3; % move down to grip 3
         0.625; % close gripper default = 0.625s
         3; % move up to grasp standoff 3
         10; % move to final standoff 5
         3; % move down to final place
         0.625; % open gripper default = 0.625s
         3; % move up to final standoff
       ];

% SET time scaling method
% 3 - cubic scaling
% 5 - quintic scaling
method = 5;

% SET type of trajectory
% 1 - Cartesian
% 2 - Screw
traj_type = 1;

configCell = { tse, 0;       % move to grasp standoff
               tsci*tces, 0; % move down to grip
               tsci*tceg, 1; % close gripper default = 0.625s
               tsci*tceg, 1; % move up to grasp standoff
               tsci*tces, 1; % move to final standoff
               tscf*tces, 1; % move to final place
               tscf*tceg, 0; % open gripper default = 0.625s
               tscf*tceg, 0; % move up to final standoff
               tscf*tces, 0; };
traj = [];

for i = 1:8
    N = round(time(i)*k*100);
    start = configCell{i,1};
    finish = configCell{i+1,1};
    tf = time(i);
    if traj_type == 1
        tempCell = CartesianTrajectory(start, finish, tf, N, method);
    elseif traj_type == 2
        tempCell = ScrewTrajectory(start, finish, tf, N, method);
    end
    for j = 1:length(tempCell)
        traj = [ traj; se3ToRow(tempCell{j}), configCell{i,2} ];
    end
end
end