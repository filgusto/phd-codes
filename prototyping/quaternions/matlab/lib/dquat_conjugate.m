function dq_out = dquat_conjugate(dq_in)
% Outputs the dual-quaternion conjugate
% 
% Input
% - dq_in: 8D input dual-quaternion
% Output
% - dq_out: 8D output dual-quaternion conjugate

% Extracting input dual-quaternion real and imaginary quaternions
[q_scalar, q_dual] = dquat_partify(dq_in);

% obtaining conjugates
q_scalar_c = q_scalar.conj;
q_dual_c = q_dual.conj;

% Mounting output dual-quaternion
dq_out = [q_scalar_c.compact.'; q_dual_c.compact.'];
end

