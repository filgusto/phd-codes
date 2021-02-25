function dquat_out = dquat_cross(dq_lhs, dq_rhs)

% extracting quaternions scalar and dual parts from input dual-quaternions
% one may remark that output are quaternion objects
[q_lhs_scalar, q_lhs_dual] = dquat_partify(dq_lhs);
[q_rhs_scalar, q_rhs_dual] = dquat_partify(dq_rhs);

% computing scalar part
q_prod_scalar = q_rhs_scalar * q_lhs_scalar;
q_prod_scalar = q_prod_scalar.normalize();

% computing dual part
q_prod_dual = (q_rhs_dual * q_lhs_scalar) + (q_rhs_scalar * q_lhs_dual);

% mounting output dual-quaternion
dquat_out = [q_prod_scalar.compact.'; q_prod_dual.compact.'];

end

