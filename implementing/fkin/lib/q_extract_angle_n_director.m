function theta = q_extract_angle(q_in)
 % Converts a rotation quaternion to an angle and director vector
    % q_in: quaternion object
    
    % normalizes the input
    q_in_norm = q_in.normalize;
    
    % converting quaternion to vectorial data type
    q_in_v = q_in_norm.compact;
    
    % expliciting quaternion components
    w = q_in_v(1);
%     x = q_in_v(2);
%     y = q_in_v(3);
%     z = q_in_v(4);
    
    % computing theta
    theta = 2 * acos(w);
    
    
end

