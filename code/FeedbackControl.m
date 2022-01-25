% takes current config, desired config, next desired config
% frame X is in terms of {s}
% ie X is actually Xsb where b is the end effector frame
% similiarly X
% Kp and Ki matrices
% timestep

function [V,Xe,integral] = FeedbackControl(X, Xd, Xdn, Kp, Ki, integral, timestep)

Xe = se3ToVec( MatrixLog6(TransInv(X)*Xd) );

Vd = se3ToVec(MatrixLog6(TransInv(Xd)*Xdn)/timestep);

ADJ_xbd = Adjoint(TransInv(X)*Xd);

integral =  Xe*timestep + integral;

% feedforward + PI control law
V = ADJ_xbd*Vd + Kp*Xe + Ki*integral;
% V =  Kp*Xe + Ki*integral;
end