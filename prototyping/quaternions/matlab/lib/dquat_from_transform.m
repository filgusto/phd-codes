function qquat_out = dquat_from_transform(transl, rot_axis, rot_angle)
% Creates a pose transform dual-quaternion from a homogeneous transform
% 
% Input
% - T: 4x4 homogeneous transform
% Ouput

% mounting the pure translation dual-quaternion
dq_tr = dquat_tr_from_vector(transl);

% mounting the pure rotation dual-quaternion
dq_rot = dquat_rot_from_axis_angle(rot_axis, rot_angle);
 
% normalizes the rotational vector if it is not
if dquat_norm(dq_rot) ~= 1
    dq_rot = dq_normalize(dq_rot);
end

% computing the transform dual-quaternion
% stopped here
qquat_out = dquat_cross(dq_tr, dq_rot);

end

