function TH = dquat_to_th(dq_in)
% Extracts homogeneous transform matrix from dual-quaternion
%
% Input
% - dq_in: 8D input dual-quaternion
% Output
% - T: 4x4 homogeneous transform

%% Preparations

% TH to receive the result
TH = eye(4);

% Normalize the input dual-quaternion
dq_in_normalized = dquat_normalize(dq_in);

% partifying the input dual-quaternion
[q_scalar, q_dual] = dquat_partify(dq_in_normalized);

%% Extracting the rotation

% obtaining the scalar quaternion components
% the same components stored in q_scalar
w = dq_in_normalized(1); %  w component
x = dq_in_normalized(2); % xi component
y = dq_in_normalized(3); % yj component
z = dq_in_normalized(4); % zk component

% storing the rotation matrix inside the homogeneus transform
% Remark that TH the below is a symmetric matrix and can be computed 
% more efficiently.
TH(1,1) = w*w + x*x - y*y - z*z;
TH(1,2) = 2*x*y - 2*w*z;
TH(1,3) = 2*x*z + 2*w*y;

TH(2,1) = 2*x*y + 2*w*z;
TH(2,2) = w*w + y*y - x*x - z*z;
TH(2,3) = 2*y*z - 2*w*x;

TH(3,1) = 2*x*z - 2*w*y;
TH(3,2) = 2*y*z + 2*w*x;
TH(3,3) = w*w + z*z - x*x - y*y;

%% Extracting the translation information

% computing the translation quaternion
q_tr = (q_dual * 2) * (q_scalar.conj);
q_tr = q_tr.compact;

% mounting it on TH
TH(1,4) = q_tr(2); % xi component
TH(2,4) = q_tr(3); % yj component
TH(3,4) = q_tr(4); % zk component

end








