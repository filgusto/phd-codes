function dq_out = dquat_normalize(dq_in)
% Normalizes a dual-quaternion
%
% Input
% - dq_in: 8D input dual-quaternion
% Output
% - dq_out: 8D output normalized dual-quaternion

% partify the dual-quaternion into two quaternions (scalar and dual parts)
[dq_in_scalar, dq_in_dual] = dquat_partify(dq_in);

% the scalar part magnitude
mag = dot(dq_in_scalar.compact, dq_in_scalar.compact);

% normalizes both parts
dq_out_scalar = dq_in_scalar * (1/mag);
dq_out_dual = dq_in_dual * (1/mag);

dq_out = [dq_out_scalar.compact.';dq_out_dual.compact.'];

end

