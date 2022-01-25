% inputs include 
% current_configuration: chassis config, arm config, wheel angles
% control_velocities: joint speeds, wheel speeds
% time_step
% control_velocity_limits: joint velocity limits, wheel velocity limits

function [updated_configuration] = NextState(current_configuration, control_velocity, control_velocity_limits, time_step)

% separating configuration variables
chassis_state = current_configuration(1:3);
joint_state = current_configuration(4:8);
wheel_state = current_configuration(9:12);

control_velocity = control_velocity(:);
control_velocity_limits = control_velocity_limits(:);

% limiting velocities
exceeded_velocities = abs(control_velocity) > abs(control_velocity_limits);
control_velocity(exceeded_velocities) = control_velocity(exceeded_velocities)./abs(control_velocity(exceeded_velocities)).*control_velocity_limits(exceeded_velocities);

% separating control variables
joint_velocity = control_velocity(5:9);
wheel_velocity = control_velocity(1:4);

joint_velocity = joint_velocity(:)';
wheel_velocity = wheel_velocity(:)';

% updating joint positions 
joint_state = joint_state + joint_velocity*time_step;

% updating wheel positions
wheel_state = wheel_state + wheel_velocity*time_step;

% updating chassis configuration

phi = chassis_state(1);
chassis_state(:) = chassis_state(:) + [1,0,0;
                                       0,cos(phi),-sin(phi);0,sin(phi),cos(phi),]*[-0.0308,0.0308,0.0308,-0.0308;
                                                                                    ones(1,4)*0.0119;
                                                                                    -0.0119,0.0119,-0.0119,0.0119]*wheel_velocity(:)*time_step;

updated_configuration = [ chassis_state(:)' , joint_state(:)', wheel_state(:)' ];

end 