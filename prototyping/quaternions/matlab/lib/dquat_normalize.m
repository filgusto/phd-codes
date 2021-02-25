function dq_out = dquat_normalize(dq_in)
% Normalizes a dual-quaternion
%
% Input
% - dq_in: 8D input dual-quaternion
% Output
% - dq_out: 8D output normalized dual-quaternion

% obtains the input norm
norm = dquat_norm(dq_in);

% normalizes the input quaternion
dq_out = dq_in./norm;
end

