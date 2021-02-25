function dq_out = dquat_from_pose(pos, q_ori)
% Encodes position and orientation information in dual-quaternion format
%
% Input
% - pos: 3D position
% Output
% - ori: 4D quaternion wxyz

% normalizes orientation, if it is not
if dquat_norm(q_ori) ~= 1
   q_ori = dquat_normalize(q_ori); 
end

% mounts the dual-quaternion
dq_out = [q_ori; 0; pos(1)/2; pos(2)/2; pos(3)/2];
% dq_out = [q_ori; 0; pos(1)/2; pos(2)/2; pos(3)/3];
end

