function norm = dquat_norm(dq_in)
% Computes the norm of a dual-quaternion
% 
% Input
% - dq_in: 8D dual quaternion 
% Output
% - norm: scalar representing the dual-quaternion magnitude
norm = sqrt(sum(dq_in.*dq_in));

%dquat_cross(dq_in, dquat_conjugate(dq_in));

end

