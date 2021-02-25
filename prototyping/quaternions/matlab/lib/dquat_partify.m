function [q_scalar, q_dual] = dquat_partify(dq_in)
% Breaks a dual-quaternion into scalar and dual parts
%
% Input
% - dq_in: 8D input dual-quaternion
% Output
% - q_scalar: Quaternion output dual-quaternion scalar
% - q_dual: Quaternion output dual-quaternion dual part

q_scalar = quaternion(dq_in(1:4).');
q_dual = quaternion(dq_in(5:end).');
end

