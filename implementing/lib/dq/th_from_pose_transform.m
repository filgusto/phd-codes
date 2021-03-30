function T = th_from_pose_transform(pos, rot_axis, rot_angle)
% Mounts a homogeneous transform matrix from a rotation matrix
% and a position vector
%
% Input
% - ROT: SO(3) rotation matrix
% - pos: 3x1 position vector
% Output
% - T: 4x4 homogeneous transform

% computing a rotation matrix from axis and angle format
R = axang2rotm([rot_axis.', rot_angle]);

% mounting the homogeneous transform matrix
T = zeros(4,4);
T(1:3,1:3) = R;
T(1:3,4) = pos;
T(4,1:3) = zeros(1,3);
T(4, 4) = 1;

end

