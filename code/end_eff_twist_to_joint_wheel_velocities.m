% function to calculate the wheel and joint speeds to achieve the inputted
% end eff twist. 
% IP = joint postions, commanded end eff twist
% OP = wheel and joint speeds

function V = end_eff_twist_to_joint_wheel_velocities(theta, Ve)

% make theta vertical
theta = theta(:);

% make Ve vertical
Ve = Ve(:);

% pseudo inverse of H0
H0_inv = [ 0.0000   -0.0000   -0.0000    0.0000;
          -0.0000    0.0000    0.0000   -0.0000;
          -0.0308    0.0308    0.0308   -0.0308;
           0.0119    0.0119    0.0119    0.0119;
          -0.0119    0.0119   -0.0119    0.0119;
           0         0         0         0      ];

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

% calculate the fixed frame {o} relative to {e} at the base of the arm thorough Foward
% kinematics in {e}
Toe = FKinBody(Moe,B,theta);

Tbe = Tbo*Toe;
Teb = TransInv(Tbe);
% calculate the adjoint to map twists in {b} to {e}
ADJ_Teb = Adjoint(Teb);

% base jacobin
Jbase = ADJ_Teb*H0_inv;

% arm jacobin
Jarm = JacobianBody(B, theta);

Je = [ Jbase Jarm ];

% set tolerance for the pseudo inverse agorithm to prevent pinv(Je) from
% having ridiculously large values.
% all numbers below this value will be set to zero in Je before pinv is
% calculated
tolerance = 1e-6;
V = pinv(Je,tolerance)*Ve;

K = eye(9).*[ 1 1 1 1 1 1 1 1 1 ];
V = K*V;

end