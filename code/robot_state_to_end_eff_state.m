function [X] = robot_state_to_end_eff_state(state)

% =========================== PARAMS =================================
% end effector frame {e} relative to fixed frame at arm base {o}
Moe = [ 1 0 0 0.033; 
        0 1 0 0;
        0 0 1 0.6546;
        0 0 0 1 ];

% each column represents a screw axis 
B = [    0         0         0         0         0;
         0   -1.0000   -1.0000   -1.0000         0;
    1.0000         0         0         0    1.0000;
         0   -0.5076   -0.3526   -0.2176         0;
    0.0330         0         0         0         0;
         0         0         0         0         0  ];

% where the centre of the bot base is wrt to {o} 
Tbo = [  1.0000         0         0    0.1662;
              0    1.0000         0         0;
              0         0    1.0000    0.0026;
              0         0         0    1.0000 ];
% ====================================================================

theta = state(4:8);
theta = theta(:);

Toe = FKinBody(Moe, B, theta);

x = state(2);
y = state(3);
phi = state(1);

Tsb = [ cos(phi), -sin(phi), 0, x;
        sin(phi),  cos(phi), 0, y;
        0,         0,        1, 0.0963;
        0,         0,        0, 1 ];

X = Tsb*Tbo*Toe;

end