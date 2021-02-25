function T = th_from_pose(pos, ori)
% Creates a homogeneous transform from a given pose
%
% Input
% - pos: 3D position vector
% - ori: Quaternion orientation
% Output
% - T: 4x4 matrix homogeneous transform encoding the input pose

% obtaining rotation matrix from input quaternion
R = quat2rotm(quaternion(ori.'));

% mounting the homogeneous transform matrix
T = zeros(4,4);
T(1:3,1:3) = R;
T(1:3,4) = pos;
T(4,1:3) = zeros(1,3);
T(4, 4) = 1;

end

