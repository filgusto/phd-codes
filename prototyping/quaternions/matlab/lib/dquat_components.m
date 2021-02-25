function [scalar,dual] = dquat_components(dq_in)
% Breaks a dual-quaternion into real and imaginary structs
%
% Input
% - dq_in: 8D input dual-quaternion
% Output
% - real: struct containing real part information
% - dual: struct containing dual part information

% breaking real part
scalar = struct();
scalar.w = dq_in(1);
scalar.x = dq_in(2);
scalar.y = dq_in(3);
scalar.z = dq_in(4);

% breaking dual part
dual = struct();
dual.w = dq_in(5);
dual.x = dq_in(6);
dual.y = dq_in(7);
dual.z = dq_in(8);

end

