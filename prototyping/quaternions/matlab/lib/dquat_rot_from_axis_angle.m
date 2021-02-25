function dquat_out = dquat_rot_from_axis_angle(r_axis, r_angle)
% Returns a pure rotation dual-quaternion given a normalized rotation axis
% and the rotation angle
% Input
% - r_axis: 3D rotation normalized rotation vector
% - r_angle: 1D rotation in radians

% aux variable
sin_angle_div_2 = sin(r_angle/2);

% mounting dual-quaternion
dquat_out = [cos(r_angle/2);...
          r_axis(1)*sin_angle_div_2; ...
          r_axis(2)*sin_angle_div_2; ...
          r_axis(3)*sin_angle_div_2; ...
          zeros(4,1)];

end

