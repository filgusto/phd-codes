function dq_ret = get_dq_frame_x_aligned(dq_in, dq_dir, dq_rot_vec)
% Finds a dq pose frame with x frame aligned to current x 
% projection over the horizontal frame
% Input
%   -dq_in: current pose
%   -dq_dir: pure translation dual quaternion containing the unit director vector
%        of the body frame. In a mobile base, this should be the front sens
%       director
%   -dq_rot_vec: pure translation dual quaternion containing the unit
%       vector about wich the director vector rotates

    % finding current x vector
    dq_dir_rotated = dq_in * dq_dir * dq_in.conj;

    % extracting x vector
    v_dir = dq_dir.extractTranslation;
    v_dir_rotated = dq_dir_rotated.extractTranslation;
    
    % finding dir vector projected on horizontal plane
    v_dir_h = [v_dir_rotated(1:2) 0];        % extracting only planar components
    v_dir_h = v_dir_h / norm(v_dir_h);       % normalizing the vector
    
    % vectorial computation    
    v_dot = dot(v_dir_h, v_dir);
    v_cross = cross(v_dir_h, v_dir);
    
    % computing angle between director and planar rotated director
    angle = -1 * sig(v_cross(3)) * acos(v_dot);
    
    
    v_rot_vec = dq_rot_vec.extractTranslation;
    
    % mounting corresponding dual quaternion
    dq_ret = DualQuaternion.pureRotation(angle, v_rot_vec);   
   
end

