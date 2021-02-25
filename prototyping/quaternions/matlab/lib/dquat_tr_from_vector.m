function dquat_out = dquat_tr_from_vector(t)
% Returns a pure translation dual-quaternion from a translation vector
% Input
% - t: translation 3D vector

dquat_out = [1; 0; 0; 0; 0; 0.5*t(1); 0.5*t(2); 0.5*t(3)];
end

