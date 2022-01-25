function compute_motion(Tsci, Tscf, current_robot_config, Tse, Kp, Ki)

% IP:
% Tsci: the initial location of the cube. expressed with a transformation
% from space frame {s} to cube frame {c}

% Tscf: the final location of the cube. expressed with a transformation
% from space frame {s} to cube frame {c}

% current_robot_config: a 12-vector composed of
%   [ phi,  chassis config
%     x,
%     y,
%     J1,  joint positions 
%     J2,
%     J3,
%     J4,
%     J5,
%     W1,  wheel positions
%     W2,
%     W3,
%     W4,
%     gripper_state is the gripper closed?
%     ]

% Tse: the start of the end-effector trajectory. expressed with a
% tranformation from space frame {s} to frame {e}

% Kp: a 6x6 diagonal matrix of gains
% Ki: a 6x6 diagonal matrix of gains

% ======== PARAMS ===========
% standoff pos
Tces = [ -1 0 0 0; 
          0 1 0 0; 
          0 0 -1 0.1; 
          0 0 0 1 ];

% grasp pos
Tceg = [ -1 0 0 0; 
          0 1 0 0; 
          0 0 -1 -0.01; 
          0 0 0 1];

% generate 'k' traj positions every 0.01s
% hence time interval between configs would be 0.01/k
k = 1;
timestep = 0.01/k;

% velocity limits for robot (rad/s)
control_velocity_limits = [ 5,5,5,5,3,3,3,3,3 ]; % wheels, joints

% joint limits (rad)
jlim = [ -Inf, Inf;
         -Inf, Inf;
         -Inf, Inf;
         -Inf, Inf;
         -Inf, Inf ];
     
% ===========================

% calculate the end-effector trajectory
end_eff_trajectory = TrajectoryGenerator(Tse,Tsci,Tscf,Tceg,Tces,k);

iterations = size(end_eff_trajectory, 1) - 1;
robot_states = zeros(iterations + 1,13);
error_acc = zeros(iterations, 6);
robot_states(1,:) = current_robot_config;
actual_robot_config = current_robot_config(1:12);
count = 0;
integral = 0;

for i = 1:iterations
    
   % gripper state
   gripper_state = end_eff_trajectory(i,13);
   
   % actual end-eff postion
   X = robot_state_to_end_eff_state(actual_robot_config(1:12));
   
   % desired end-eff position
   Xd = row2se3(end_eff_trajectory(i,1:12));
   
   % next desired end-eff position
   Xdn = row2se3(end_eff_trajectory(i+1,1:12));
   
   % calculate twist in end-eff frame to go from actual to desired position
   [Ve,Xe,integral] = FeedbackControl(X,Xd,Xdn,Kp,Ki,integral,timestep);
   
   % accumulate the errors
   error_acc(i,:) = Xe(:)';
   
   % calculate the wheel and joint velocities to achieve this twist.
   control_velocity = end_eff_twist_to_joint_wheel_velocities(actual_robot_config(4:8),Ve);

   % calculate the effect of the velocities on robot configuration if
   % applied for 'timestamp' seconds
   actual_robot_config = NextState(actual_robot_config, control_velocity, control_velocity_limits, timestep);
   actual_robot_config(4:8) = limit_joints(actual_robot_config(4:8),jlim);

   count = count + 1;
   if count == k
       
       % save every kth configuration
       robot_states(i+1,:) = [actual_robot_config, gripper_state];
       
       % accumulate the errors
       error_acc(i,:) = Xe(:)';
       
       count = 0;
   end
   
end

fprintf("saved robot states and error in pwd\n")
csvwrite("robot_states.csv", robot_states);
csvwrite("error.csv", error_acc);
end