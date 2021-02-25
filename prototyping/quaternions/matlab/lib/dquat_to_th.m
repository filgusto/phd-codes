function T = dquat_to_th(dq_in)
% Extracts homogeneous transform matrix from dual-quaternion
%
% Input
% - dq_in: 8D input dual-quaternion
% Output
% - T: 4x4 homogeneous transform

% normalize input dual quaternion if it is not
if dquat_norm(dq_in) ~= 1
    dq_in = dquat_normalize(dq_in);
end

% --Extracting rotation component

% extracting scalar(s) and dual (d) components from input dual-quaternion
[s, d] = dquat_components(dq_in);

% computing rotation matrix from quaternion components
R = eye(3);
R(1,1) = (s.w)^2 + (s.x)^2 - (s.y)^2 - (s.z)^2;
R(1,2) = 2*s.x*s.y + 2*s.w*s.z;
R(1,3) = 2*s.x*s.z - 2*s.w*s.y;

R(2,1) = 2*s.x*s.y - 2*s.w*s.z;
R(2,2) = (s.w)^2 + (s.y)^2 - (s.x)^2 - (s.z)^2;
R(2,3) = 2*s.y*s.z + 2*s.w*s.x;

R(3,1) = 2*s.x*s.z + 2*s.w*s.y;
R(3,2) = 2*s.y*s.z - 2*s.w*s.x;
R(3,3) = (s.w)^2 + (s.z)^2 - (s.x)^2 - (s.y)^2;

% --Computing translation information

% extracting scalar and dual quaternions from input dual-quaternion
[q_s, q_d] = dquat_partify(dq_in);

% computing equivalent quaternon
q_trans = 2 * q_d * q_s.conj;
q_trans = q_trans.compact;

% translating obtained quaternion into 3D vector
t = q_trans(2:4);

% --Mounting output th
T = eye(4);
T(1:3,1:3) = R;
T(1:3,4) = t.';

end








